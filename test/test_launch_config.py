'''
Unit tests of the launch_config settings and of their effect on the nodes of SimpleLauncher.node()
Run with: python3 -m pytest test/test_launch_config.py
'''

import pytest
import yaml
from launch import LaunchContext
from launch.utilities import perform_substitutions

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
