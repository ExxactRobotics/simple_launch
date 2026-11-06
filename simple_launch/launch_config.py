'''
Launch settings of the nodes, read from the YAML file given by the launch_config launch argument.

The file maps patterns on the full node name (fnmatch, where * also matches /) to settings:

    nodes:
      '*':
        log_level: info
      /navigation/*:
        log_level: debug

Rules apply from top to bottom. For each setting, the last rule that matches the node and
names that setting wins: put the catch-all first, then groups, then single nodes.

An unknown setting or a value of the wrong type stops the launch. The rules that matched no
node are logged when the launch shuts down, since that is how a mistyped pattern shows.
'''

import fnmatch
import os
import re
import shutil
import sys
from dataclasses import dataclass, field, fields
from typing import Dict, List, Optional, Set, Text, Tuple

from launch import Substitution
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import evaluate_condition_expression
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

from .param_overrides import full_node_name, launch_global, read_yaml_mapping

ARG_NAME = 'launch_config'
# launch argument: true logs the settings of every node and starts none
DRY_RUN_ARG = 'dry_run'

LOG_LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')


@dataclass
class NodeSettings:
    '''
    Launch settings of one node. A setting no rule names keeps its default.
    '''

    # a level for the whole process, or {logger: level} where 'default' is the whole process
    log_level: object = field(default=None, metadata={'type': (str, dict), 'choices': LOG_LEVELS})
    # false leaves the node out of the launch
    enabled: bool = field(default=True, metadata={'type': bool})
    # true shuts the whole launch down when the node exits
    required: bool = field(default=False, metadata={'type': bool})
    # true starts the node again when it dies, after respawn_delay seconds
    respawn: Optional[bool] = field(default=None, metadata={'type': bool})
    respawn_delay: Optional[float] = field(default=None, metadata={'type': (int, float)})
    # command the node is started with, e.g. 'gdb -batch -ex run -ex bt --args'
    prefix: Optional[Text] = field(default=None, metadata={'type': str})
    # true or {port, host}: the node waits for a debugger before main(), see debug_prefix()
    debug: object = field(default=None, metadata={'type': (bool, dict), 'keys': ('port', 'host')})


def load_rules(path: Text) -> List[Tuple[Text, dict]]:
    '''
    Returns the (pattern, settings) rules of a launch config file.
    An empty path or file gives no rules. Raises RuntimeError on a missing file or a wrong
    layout, such as a top-level key other than 'nodes', so that a misplaced rule is not ignored.
    '''
    if not path:
        return []
    path = os.path.abspath(os.path.expanduser(path))
    content = read_yaml_mapping(path, ARG_NAME)
    unknown = sorted(str(key) for key in content if key != 'nodes')
    if unknown:
        raise RuntimeError(f'{ARG_NAME}: {path} has top-level keys {unknown}, only "nodes" is known')
    nodes = content.get('nodes') or {}
    if not isinstance(nodes, dict) or not all(isinstance(s, dict) for s in nodes.values()):
        raise RuntimeError(f'{ARG_NAME}: {path} must map "nodes" to pattern: settings entries')

    rules = [(str(pattern), settings) for pattern, settings in nodes.items()]
    for pattern, settings in rules:
        check_settings(settings, f'{ARG_NAME}: {path}, rule {pattern!r}')
    return rules


def check_settings(settings: dict, where: Text) -> None:
    '''
    Raises RuntimeError if a rule names an unknown setting or gives one a value of the wrong type
    '''
    known = {f.name: f.metadata for f in fields(NodeSettings)}
    unknown = sorted(set(settings) - set(known))
    if unknown:
        raise RuntimeError(f'{where}: unknown settings {unknown}, known ones are {sorted(known)}')
    for key, value in settings.items():
        expected = known[key].get('type')
        types = expected if isinstance(expected, tuple) else (expected,)
        # YAML true is also an int, but no number setting takes it
        wrong_bool = isinstance(value, bool) and bool not in types
        if expected is not None and (wrong_bool or not isinstance(value, types)):
            names = '/'.join(t.__name__ for t in types)
            raise RuntimeError(f'{where}: {key} must be a {names}, not {value!r}')
        keys = known[key].get('keys')
        if keys is not None and isinstance(value, dict) and set(value) - set(keys):
            raise RuntimeError(f'{where}: {key} takes only {list(keys)}, not {sorted(value)}')
        choices = known[key].get('choices')
        for choice in (value.values() if isinstance(value, dict) else [value]):
            if choices is not None and str(choice).lower() not in choices:
                raise RuntimeError(f'{where}: {key} must be one of {list(choices)}, not {choice!r}')


def unmatched_rules(state: Dict[Text, Tuple[list, Set[Text]]]) -> List[Tuple[Text, Text]]:
    '''
    Returns the (file, pattern) rules of a launch that matched no node
    '''
    return [(path, pattern) for path, (rules, matched) in state.items() for pattern, _ in rules
            if pattern not in matched]


def launch_state(context: LaunchContext) -> Dict[Text, Tuple[list, Set[Text]]]:
    '''
    Returns the rules this launch read, per file, with the patterns that matched a node.
    Kept on the launch context, so that each launch starts afresh. The rules that matched
    no node are logged once, when the launch shuts down.
    '''
    def create():
        state = {}
        logger = get_logger('simple_launch')
        context.register_event_handler(OnShutdown(
            on_shutdown=lambda event, context: [
                logger.info(f'{ARG_NAME}: rule {pattern!r} of {path} matched no node')
                for path, pattern in unmatched_rules(state)],
            handle_once=True))
        return state
    return launch_global(context, 'simple_launch_config', create)


def resolve(rules: List[Tuple[Text, dict]], full_name: Text) -> Tuple[NodeSettings, List[Text]]:
    '''
    Returns the settings of the node of this full name, the last matching rule winning per setting,
    with the patterns of the rules that matched it, in file order
    '''
    settings = NodeSettings()
    patterns = []
    for pattern, values in rules:
        if fnmatch.fnmatchcase(full_name, pattern):
            patterns.append(pattern)
            for key, value in values.items():
                setattr(settings, key, value)
    return settings, patterns


def describe(full_name: Text, settings: NodeSettings, rules: List[Text]) -> Text:
    '''
    Returns a one-line summary of the settings of a node: those that differ from the default,
    then the patterns of the rules that matched
    '''
    default = NodeSettings()
    changed = [f'{f.name}={getattr(settings, f.name)!r}' for f in fields(NodeSettings)
               if getattr(settings, f.name) != getattr(default, f.name)]
    return f'{full_name}: {" ".join(changed) or "defaults"} (rules: {", ".join(rules) or "none"})'


def is_dry_run(context: LaunchContext) -> bool:
    '''
    Returns whether the dry_run launch argument is true, read like any launch condition
    '''
    return evaluate_condition_expression(
        context, [LaunchConfiguration(DRY_RUN_ARG, default='false')])


def log_summary(context: LaunchContext, summary: Text) -> bool:
    '''
    Logs the settings summary of a node, at info level in a dry run and at debug level otherwise.
    Returns whether the launch is a dry run.
    '''
    dry_run = is_dry_run(context)
    logger = get_logger('simple_launch')
    (logger.info if dry_run else logger.debug)(f'{ARG_NAME}: {summary}')
    return dry_run


def settings_of(context: LaunchContext, full_name: Text) -> Tuple[NodeSettings, List[Text]]:
    '''
    Returns the settings of a node from the launch_config file of this launch, with the patterns
    of the rules that matched it, and records those
    '''
    path = perform_substitutions(context, [LaunchConfiguration(ARG_NAME, default='')])
    if not path:
        return NodeSettings(), []
    path = os.path.abspath(os.path.expanduser(path))
    state = launch_state(context)
    if path not in state:
        state[path] = (load_rules(path), set())
    rules, matched = state[path]
    settings, patterns = resolve(rules, full_name)
    matched.update(patterns)
    return settings, patterns


def log_args(settings: NodeSettings, logger: Optional[Text] = None) -> List[List[TextSubstitution]]:
    '''
    Returns the --log-level arguments of a node's settings, none if they set no level.
    The 'default' level is the one of the whole process, as for a node alone in it, or of the
    given logger when the node shares its process. Other names are loggers of the process.
    '''
    levels = settings.log_level
    if not levels:
        return []
    if not isinstance(levels, dict):
        levels = {'default': levels}
    args = []
    for name, level in levels.items():
        name = logger if name == 'default' else name
        args += ['--log-level', level if name is None else f'{name}:={level}']
    return [[TextSubstitution(text=arg)] for arg in args]


class NodePrefix(Substitution):
    '''
    Prefix of a ConfiguredNode: the one of its settings, else the one its launch file gives,
    else the launch-prefix launch configuration filtered by launch-prefix-filter, as launch does
    '''

    def __init__(self, node, given):
        super().__init__()
        self.__node = node
        self.__given = None if given is None else normalize_to_list_of_substitutions(given)

    def describe(self) -> Text:
        return 'NodePrefix()'

    def perform(self, context: LaunchContext) -> Text:
        settings = self.__node.settings
        if settings is not None and settings.prefix:
            return settings.prefix
        if self.__given is not None:
            return perform_substitutions(context, self.__given)
        prefix_filter = context.launch_configurations.get('launch-prefix-filter', '')
        executable = os.path.basename(perform_substitutions(
            context, normalize_to_list_of_substitutions(self.__node.node_executable)))
        if prefix_filter and re.match(prefix_filter, executable) is None:
            return ''
        return context.launch_configurations.get('launch-prefix', '')


class NodeRespawn(Substitution):
    '''
    Respawn flag of a ConfiguredNode: the one of its settings, else the one its launch file gives
    '''

    def __init__(self, node, given):
        super().__init__()
        self.__node = node
        self.__given = normalize_to_list_of_substitutions(
            str(given).lower() if isinstance(given, bool) else given)

    def describe(self) -> Text:
        return 'NodeRespawn()'

    def perform(self, context: LaunchContext) -> Text:
        settings = self.__node.settings
        if settings is not None and settings.respawn is not None:
            return str(settings.respawn).lower()
        return perform_substitutions(context, self.__given)


def debug_prefix(executable: Text, debug) -> Tuple[Text, Text]:
    '''
    Returns the prefix that makes a node wait for a debugger before main(), and the host:port
    the debugger connects to: gdbserver for a compiled executable, debugpy for a Python one,
    each on its usual port unless one is given. The default host keeps the port off the network.
    The executable is the one the node runs: a path, or a name looked up on PATH.
    '''
    debug = debug if isinstance(debug, dict) else {}
    path = shutil.which(executable)
    if path is None:
        raise RuntimeError(f'{ARG_NAME}: cannot debug {executable}, no such executable')
    with open(path, 'rb') as f:
        compiled = f.read(4) == b'\x7fELF'
    address = f'{debug.get("host", "127.0.0.1")}:{debug.get("port", 3000 if compiled else 5678)}'
    if compiled:
        return f'gdbserver --once {address}', address
    # the interpreter of this launch, which is the one of ROS
    return f'{sys.executable} -m debugpy --listen {address} --wait-for-client', address


def claim_debug_address(context: LaunchContext, address: Text, full_name: Text) -> None:
    '''
    Records that a node waits for its debugger at this address, and raises RuntimeError
    if another node of the launch already does: only one of them could listen there
    '''
    other = launch_global(context, 'simple_launch_debug', dict).setdefault(address, full_name)
    if other != full_name:
        raise RuntimeError(f'{ARG_NAME}: {full_name} and {other} both wait for a debugger at '
                           f'{address}, give one of them its own debug port')


class ConfiguredNode(Node):
    '''
    Node whose launch settings come from the launch_config file. They are resolved when the
    node starts, because only then is its full name known (namespaces pushed by including files).
    '''

    def __init__(self, *, executable, package=None, **kwargs):
        # the settings of this node, set when it starts
        self.settings = None
        prefix = NodePrefix(self, kwargs.pop('prefix', None))
        respawn = NodeRespawn(self, kwargs.pop('respawn', False))
        super().__init__(executable=executable, package=package, prefix=prefix, respawn=respawn,
                         **kwargs)

    def full_name(self, context: LaunchContext) -> Text:
        '''
        Returns the full name of the node, using its executable when it is given no name
        '''
        self._perform_substitutions(context)
        executable = os.path.basename(perform_substitutions(
            context, normalize_to_list_of_substitutions(self.node_executable)))
        return full_node_name(self, executable)

    def _configure(self, context: LaunchContext) -> Tuple[Text, List[Text]]:
        '''
        Resolves the settings of this node and applies them to its command. Returns the full
        name of the node and the patterns of the rules that matched it.
        '''
        full_name = self.full_name(context)
        self.settings, patterns = settings_of(context, full_name)

        # an empty {port, host} mapping asks for the defaults
        if self.settings.debug not in (None, False):
            # the command launch_ros runs, which finds the executable of a package as it does
            executable = perform_substitutions(context, self.cmd[0])
            self.settings.prefix, address = debug_prefix(executable, self.settings.debug)
            claim_debug_address(context, address, full_name)
            # a node stopped in a debugger must not come back by itself, and its debugger
            # detaching must not stop the launch
            self.settings.respawn = False
            self.settings.required = False
            get_logger('simple_launch').info(
                f'{ARG_NAME}: {full_name} waits for a debugger: {self.settings.prefix}')

        if self.settings.respawn_delay is not None:
            # launch takes the delay as a plain number at construction only, so it is replaced
            # in place, and an attribute that launch renamed must not be silently ignored
            if not hasattr(self, '_ExecuteLocal__respawn_delay'):
                raise RuntimeError(f'{ARG_NAME}: this version of launch cannot set respawn_delay')
            self._ExecuteLocal__respawn_delay = float(self.settings.respawn_delay)

        # given after the node's own arguments, so that it wins over them
        self.cmd.extend(log_args(self.settings))
        return full_name, patterns

    def execute(self, context: LaunchContext):
        full_name, patterns = self._configure(context)
        if log_summary(context, describe(full_name, self.settings, patterns)):
            return None
        if not self.settings.enabled:
            get_logger('simple_launch').info(f'{ARG_NAME}: {full_name} not started, enabled is false')
            return None
        self._supervise(context, full_name)
        return super().execute(context)

    def _supervise(self, context: LaunchContext, full_name: Text) -> None:
        '''
        Shuts the launch down when this node exits, if its settings say it is required
        '''
        if self.settings.required:
            reason = f'{full_name} exited and is required'
            context.register_event_handler(OnProcessExit(
                target_action=self, on_exit=[Shutdown(reason=reason)]))
