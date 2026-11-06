'''
Unit tests of the launch_config settings and of their effect on the nodes of SimpleLauncher.node()
Run with: python3 -m pytest test/test_launch_config.py
'''

import sys

import pytest
import yaml
from launch import LaunchContext
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.substitutions import find_package

from simple_launch import SimpleLauncher
from simple_launch import launch_config
from simple_launch.launch_config import (ConfiguredNode, describe, load_rules,
                                         resolve, unmatched_rules)


def write_config(tmp_path, nodes, name='launch.yaml'):
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'nodes': nodes}, sort_keys=False))
    return str(path)


def context_for(config, namespace=None):
    context = LaunchContext()
    context.launch_configurations['launch_config'] = config
    if namespace is not None:
        context.launch_configurations['ros_namespace'] = namespace
    return context


def command(node, context):
    # Node.execute() fills these in before the command is expanded
    context.extend_locals({'ros_specific_arguments': {'name': '__node:=n', 'ns': '__ns:=/ns'}})
    return [perform_substitutions(context, part) for part in node.cmd[1:]]


def test_last_matching_rule_wins_per_setting(tmp_path):
    rules = load_rules(write_config(tmp_path, {
        '*': {'log_level': 'info'},
        '/navigation/*': {'log_level': 'debug'},
        '/navigation/planner_server': {},
    }))

    assert resolve(rules, '/navigation/controller_server')[0].log_level == 'debug'
    assert resolve(rules, '/brain/mode_fsm')[0].log_level == 'info'
    # a rule that names no setting keeps the earlier values
    planner, patterns = resolve(rules, '/navigation/planner_server')
    assert planner.log_level == 'debug'
    assert patterns == ['*', '/navigation/*', '/navigation/planner_server']


def test_no_file_gives_no_rules():
    assert load_rules('') == []
    assert resolve([], '/a')[0].log_level is None


def test_file_that_is_not_a_mapping_is_refused(tmp_path):
    path = tmp_path / 'list.yaml'
    path.write_text(yaml.safe_dump(['*']))

    with pytest.raises(RuntimeError, match='not a YAML mapping'):
        load_rules(str(path))


def test_wrong_layout_is_refused(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump({'nodes': {'*': 'debug'}}))

    with pytest.raises(RuntimeError, match='pattern: settings'):
        load_rules(str(path))
    with pytest.raises(RuntimeError, match='not found'):
        load_rules(str(tmp_path / 'missing.yaml'))


@pytest.mark.parametrize('content', [{'*': {'log_level': 'debug'}}, {'node': {'*': {}}}])
def test_rules_outside_nodes_are_refused(tmp_path, content):
    path = tmp_path / 'misplaced.yaml'
    path.write_text(yaml.safe_dump(content))

    with pytest.raises(RuntimeError, match='only "nodes"'):
        load_rules(str(path))


def test_empty_file_gives_no_rules(tmp_path):
    path = tmp_path / 'empty.yaml'
    path.write_text('')

    assert load_rules(str(path)) == []


def test_sl_node_creates_a_configured_node():
    assert isinstance(SimpleLauncher().node('pkg', 'exe'), ConfiguredNode)


@pytest.mark.parametrize('name, namespace, full_name', [
    ('ctrl', '/navigation', '/navigation/ctrl'),
    (None, '/navigation', '/navigation/exe'),
    ('ctrl', None, '/ctrl'),
])
def test_full_name_includes_the_pushed_namespace(name, namespace, full_name):
    node = ConfiguredNode(package='pkg', executable='exe', name=name)

    assert node.full_name(context_for('', namespace)) == full_name


def test_log_level_is_appended_after_the_node_arguments(tmp_path):
    config = write_config(tmp_path, {'/navigation/*': {'log_level': 'debug'}})
    node = ConfiguredNode(package='pkg', executable='exe', name='ctrl',
                          ros_arguments=['--log-level', 'warn'])
    context = context_for(config, '/navigation')

    node._configure(context)

    assert command(node, context)[-2:] == ['--log-level', 'debug']
    assert '--log-level' in command(node, context)[:-2]


def test_node_without_matching_rule_is_unchanged(tmp_path):
    config = write_config(tmp_path, {'/navigation/*': {'log_level': 'debug'}})
    node = ConfiguredNode(package='pkg', executable='exe', name='mode_fsm')
    context = context_for(config, '/brain')

    node._configure(context)

    assert '--log-level' not in command(node, context)


def test_log_level_mapping_sets_the_process_and_named_loggers(tmp_path):
    config = write_config(tmp_path, {
        '/motion/*': {'log_level': 'warn'},
        '/motion/velocity_limiter': {'log_level': {'default': 'info', 'rcl': 'error'}},
    })
    node = ConfiguredNode(package='pkg', executable='exe', name='velocity_limiter')
    context = context_for(config, '/motion')

    node._configure(context)

    # the later rule replaces the whole value
    assert command(node, context)[-4:] == ['--log-level', 'info', '--log-level', 'rcl:=error']


@pytest.mark.parametrize('settings, message', [
    ({'log_levle': 'debug'}, 'unknown settings'),
    ({'log_level': 3}, 'must be a str'),
    ({'log_level': 'loud'}, 'must be one of'),
])
def test_wrong_setting_is_refused(tmp_path, settings, message):
    with pytest.raises(RuntimeError, match=message):
        load_rules(write_config(tmp_path, {'*': settings}))


def test_log_level_mapping_checks_each_level(tmp_path):
    with pytest.raises(RuntimeError, match="not 'loud'"):
        load_rules(write_config(tmp_path, {'*': {'log_level': {'rcl': 'loud'}}}))


def test_level_case_is_free(tmp_path):
    assert load_rules(write_config(tmp_path, {'*': {'log_level': 'WARN'}}))


def test_rules_that_matched_no_node_are_listed(tmp_path):
    config = write_config(tmp_path, {
        '/navigation/*': {'log_level': 'debug'},
        '/navigaton/*': {'log_level': 'debug'},
    })
    context = context_for(config, '/navigation')

    ConfiguredNode(package='pkg', executable='exe', name='ctrl')._configure(context)

    state = launch_config.launch_state(context)
    assert [pattern for _, pattern in unmatched_rules(state)] == ['/navigaton/*']


@pytest.mark.parametrize('value', ['true', 'True', '1'])
def test_dry_run_starts_nothing(tmp_path, value):
    context = context_for(write_config(tmp_path, {'*': {'log_level': 'info'}}), '/brain')
    context.launch_configurations['dry_run'] = value
    node = ConfiguredNode(package='pkg', executable='exe', name='mode_fsm')

    assert node.execute(context) is None
    assert node.process_details is None


def test_dry_run_of_a_wrong_value_stops_the_launch(tmp_path):
    from launch.conditions import InvalidConditionExpressionError
    context = context_for(write_config(tmp_path, {'*': {}}), '/brain')
    context.launch_configurations['dry_run'] = 'yes'

    with pytest.raises(InvalidConditionExpressionError):
        ConfiguredNode(package='pkg', executable='exe', name='mode_fsm').execute(context)


def test_describe_lists_the_changed_settings_and_the_rules(tmp_path):
    rules = load_rules(write_config(tmp_path, {'*': {}, '/brain/*': {'log_level': 'debug'}}))

    assert describe('/brain/mode_fsm', *resolve(rules, '/brain/mode_fsm')) == \
        "/brain/mode_fsm: log_level='debug' (rules: *, /brain/*)"
    assert describe('/a', *resolve([], '/a')) == '/a: defaults (rules: none)'


def prefix_of(node, context):
    return perform_substitutions(context, node.process_description.prefix)


def test_prefix_of_the_settings_wins(tmp_path):
    config = write_config(tmp_path, {'/navigation/ctrl': {'prefix': 'gdb -batch -ex run -ex bt --args'}})
    node = ConfiguredNode(package='pkg', executable='exe', name='ctrl', prefix='valgrind')
    context = context_for(config, '/navigation')

    node._configure(context)

    assert prefix_of(node, context) == 'gdb -batch -ex run -ex bt --args'


def test_prefix_falls_back_to_the_launch_file_then_launch_prefix(tmp_path):
    config = write_config(tmp_path, {'/other': {'prefix': 'perf record'}})
    context = context_for(config, '/navigation')
    context.launch_configurations['launch-prefix'] = 'xterm -e'

    given = ConfiguredNode(package='pkg', executable='exe', name='ctrl', prefix='valgrind')
    given._configure(context)
    assert prefix_of(given, context) == 'valgrind'

    plain = ConfiguredNode(package='pkg', executable='exe', name='ctrl')
    plain._configure(context)
    assert prefix_of(plain, context) == 'xterm -e'

    context.launch_configurations['launch-prefix-filter'] = 'other_exe'
    assert prefix_of(plain, context) == ''


def test_disabled_node_is_not_started(tmp_path):
    config = write_config(tmp_path, {'/navigation/*': {'enabled': False}})
    node = ConfiguredNode(package='pkg', executable='exe', name='ctrl')
    context = context_for(config, '/navigation')

    assert node.execute(context) is None
    assert node.process_details is None
    assert not node.settings.enabled


def test_enabled_must_be_a_boolean(tmp_path):
    with pytest.raises(RuntimeError, match='must be a bool'):
        load_rules(write_config(tmp_path, {'*': {'enabled': 'no'}}))


@pytest.mark.parametrize('required', [True, False])
def test_required_node_shuts_the_launch_down(tmp_path, required):
    config = write_config(tmp_path, {'/brain/*': {'required': required}})
    node = ConfiguredNode(package='pkg', executable='exe', name='mode_fsm')
    context = context_for(config, '/brain')

    full_name, _ = node._configure(context)
    node._supervise(context, full_name)

    handlers = [h for h in context._event_handlers if isinstance(h, OnProcessExit)]
    assert len(handlers) == (1 if required else 0)


@pytest.mark.parametrize('given, rule, expected', [
    (False, {'respawn': True}, 'true'),
    (True, {'respawn': False}, 'false'),
    (True, {}, 'true'),
    (False, {}, 'false'),
])
def test_respawn_of_the_settings_wins(tmp_path, given, rule, expected):
    config = write_config(tmp_path, {'/sensing/*': rule})
    node = ConfiguredNode(package='pkg', executable='exe', name='camera', respawn=given)
    context = context_for(config, '/sensing')

    node._configure(context)

    # launch keeps the flag as a list of substitutions until the node starts
    assert perform_substitutions(context, node._ExecuteLocal__respawn) == expected


def test_respawn_delay_is_set(tmp_path):
    config = write_config(tmp_path, {'/sensing/*': {'respawn': True, 'respawn_delay': 2}})
    node = ConfiguredNode(package='pkg', executable='exe', name='camera', respawn_delay=0.5)
    context = context_for(config, '/sensing')

    node._configure(context)

    assert node._ExecuteLocal__respawn_delay == 2.0


def test_respawn_delay_stops_the_launch_if_launch_renamed_it(tmp_path):
    config = write_config(tmp_path, {'/sensing/*': {'respawn_delay': 2}})
    node = ConfiguredNode(package='pkg', executable='exe', name='camera')
    del node._ExecuteLocal__respawn_delay

    with pytest.raises(RuntimeError, match='cannot set respawn_delay'):
        node._configure(context_for(config, '/sensing'))


def test_respawn_delay_must_be_a_number(tmp_path):
    with pytest.raises(RuntimeError, match='must be a int/float'):
        load_rules(write_config(tmp_path, {'*': {'respawn_delay': 'soon'}}))


def test_respawn_delay_refuses_a_boolean(tmp_path):
    with pytest.raises(RuntimeError, match='must be a int/float'):
        load_rules(write_config(tmp_path, {'*': {'respawn_delay': True}}))


@pytest.fixture
def fake_package(tmp_path, monkeypatch):
    lib = tmp_path / 'lib' / 'pkg'
    lib.mkdir(parents=True)
    (lib / 'cpp_exe').write_bytes(b'\x7fELF' + bytes(12))
    (lib / 'py_exe').write_text('#!/usr/bin/python3\n')
    for exe in lib.iterdir():
        exe.chmod(0o755)
    # launch_ros finds the executables of a package under <prefix>/lib/<package>
    monkeypatch.setattr(find_package, 'get_package_prefix', lambda package: str(tmp_path))
    return lib


@pytest.mark.parametrize('executable, debug, prefix', [
    ('cpp_exe', True, 'gdbserver --once 127.0.0.1:3000'),
    ('py_exe', True, f'{sys.executable} -m debugpy --listen 127.0.0.1:5678 --wait-for-client'),
    ('cpp_exe', {'port': 3001, 'host': '0.0.0.0'}, 'gdbserver --once 0.0.0.0:3001'),
])
def test_debug_picks_the_debugger_and_stops_respawn_and_required(
        tmp_path, fake_package, executable, debug, prefix):
    config = write_config(tmp_path, {'/brain/*': {'respawn': True, 'required': True, 'debug': debug}})
    node = ConfiguredNode(package='pkg', executable=executable, name='mode_fsm')
    context = context_for(config, '/brain')

    node._configure(context)

    assert prefix_of(node, context) == prefix
    assert perform_substitutions(context, node._ExecuteLocal__respawn) == 'false'
    assert not node.settings.required


@pytest.mark.parametrize('package, executable', [
    (LaunchConfiguration('pkg'), 'cpp_exe'),
    (None, 'LIB/cpp_exe'),
])
def test_debug_finds_the_executable_the_node_runs(tmp_path, fake_package, package, executable):
    config = write_config(tmp_path, {'/brain/*': {'debug': True}})
    executable = executable.replace('LIB', str(fake_package))
    node = ConfiguredNode(package=package, executable=executable, name='mode_fsm')
    context = context_for(config, '/brain')
    context.launch_configurations['pkg'] = 'pkg'

    node._configure(context)

    assert prefix_of(node, context) == 'gdbserver --once 127.0.0.1:3000'


def test_debug_of_a_missing_executable_is_refused(tmp_path):
    config = write_config(tmp_path, {'/brain/*': {'debug': True}})
    node = ConfiguredNode(executable=str(tmp_path / 'missing'), name='mode_fsm')

    with pytest.raises(RuntimeError, match='cannot debug'):
        node._configure(context_for(config, '/brain'))


def test_two_nodes_cannot_wait_at_the_same_debug_address(tmp_path, fake_package):
    config = write_config(tmp_path, {'/brain/*': {'debug': True},
                                     '/brain/bt': {'debug': {'port': 3001}}})
    context = context_for(config, '/brain')

    ConfiguredNode(package='pkg', executable='cpp_exe', name='mode_fsm')._configure(context)
    # the same node again, and another one on its own port, are fine
    ConfiguredNode(package='pkg', executable='cpp_exe', name='mode_fsm')._configure(context)
    ConfiguredNode(package='pkg', executable='cpp_exe', name='bt')._configure(context)

    with pytest.raises(RuntimeError, match='/brain/other and /brain/mode_fsm both wait'):
        ConfiguredNode(package='pkg', executable='cpp_exe', name='other')._configure(context)


def test_debug_takes_only_port_and_host(tmp_path):
    with pytest.raises(RuntimeError, match='takes only'):
        load_rules(write_config(tmp_path, {'*': {'debug': {'prot': 3000}}}))


def test_empty_debug_mapping_uses_the_defaults(tmp_path, fake_package):
    config = write_config(tmp_path, {'/brain/*': {'debug': {}}})
    node = ConfiguredNode(package='pkg', executable='cpp_exe', name='mode_fsm')
    context = context_for(config, '/brain')

    node._configure(context)

    assert prefix_of(node, context).startswith('gdbserver --once 127.0.0.1:')
