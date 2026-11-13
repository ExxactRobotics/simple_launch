'''
Unit tests of the param_overrides merge and of its place in SimpleLauncher.node()
Run with: python3 -m pytest test/test_param_overrides.py
'''

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from simple_launch import SimpleLauncher, param_overrides
from simple_launch.param_overrides import (MergedOverrides, full_node_name, launch_overrides,
                                           merge_override_files, node_sections)


def write(tmp_path, name, content):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(content))
    return str(path)


def section(**params):
    return {'ros__parameters': params}


def test_later_file_wins_and_nested_keys_merge(tmp_path):
    a = write(tmp_path, 'a.yaml', {'/**/node': section(rate=1.0, kept='a')})
    b = write(tmp_path, 'b.yaml', {'/**/node': section(rate=2.0), '/**/other': section(x=1)})

    assert merge_override_files([a, b]) == {
        '/**/node': section(rate=2.0, kept='a'),
        '/**/other': section(x=1)}
    assert merge_override_files([b, a])['/**/node'] == section(rate=1.0, kept='a')


def test_list_is_replaced_whole(tmp_path):
    a = write(tmp_path, 'a.yaml', {'/**/node': section(names=['x', 'y'])})
    b = write(tmp_path, 'b.yaml', {'/**/node': section(names=['z'])})

    assert merge_override_files([a, b])['/**/node'] == section(names=['z'])


@pytest.mark.parametrize('key', ['node', '/**/navigation/node', '/**/'])
def test_key_other_than_wildcard_and_node_name_is_refused(tmp_path, key):
    bad = write(tmp_path, 'bad.yaml', {key: section(rate=1.0)})

    with pytest.raises(RuntimeError, match='/\\*\\*/<node>'):
        merge_override_files([bad])


def test_missing_file_is_refused(tmp_path):
    with pytest.raises(RuntimeError, match='not found'):
        merge_override_files([str(tmp_path / 'missing.yaml')])


def test_substitutions_are_performed_in_the_files(tmp_path, monkeypatch):
    monkeypatch.setenv('SL_TEST_DIR', '/data')
    path = tmp_path / 'subst.yaml'
    path.write_text(
        '/**/node:\n'
        '  ros__parameters:\n'
        '    tree: $(find-pkg-share rclpy)/trees/main.xml\n'
        '    bags: $(env SL_TEST_DIR)/bags\n'
        '    mode: $(var mode)\n'
        '    engine: /opt/models/model.engine\n')
    context = LaunchContext()
    context.launch_configurations['mode'] = 'fast'

    params = launch_overrides(context, [str(path)])['/**/node']['ros__parameters']

    assert params == {'tree': get_package_share_directory('rclpy') + '/trees/main.xml',
                      'bags': '/data/bags', 'mode': 'fast',
                      'engine': '/opt/models/model.engine'}


def test_no_override_gives_a_file_that_sets_nothing():
    assert node_sections({}) == {'/**/**': section()}


def test_sections_are_keyed_as_no_other_file_does_wildcard_first():
    # rcl merges a key already seen in an earlier file into that earlier section, which
    # the launch file's own parameters then win over
    merged = {'/**/talker': section(rate=2.0), '/**': section(rate=1.0)}

    sections = node_sections(merged)

    assert list(sections) == ['/**/**', '/**/**/talker']
    assert sections['/**/**/talker'] == section(rate=2.0)


def test_files_are_read_once_per_launch(tmp_path):
    a = write(tmp_path, 'a.yaml', {'/**/node': section(rate=1.0)})
    context = LaunchContext()

    first = launch_overrides(context, [a])
    write(tmp_path, 'a.yaml', {'/**/node': section(rate=2.0)})

    assert launch_overrides(context, [a]) is first
    assert launch_overrides(LaunchContext(), [a]) == {'/**/node': section(rate=2.0)}


@pytest.mark.parametrize('parameters', [None, {'a': 1}, [{'a': 1}, 'file.yaml']])
@pytest.mark.parametrize('sim_time', [None, False])
def test_node_gets_the_overrides_last(parameters, sim_time):
    sl = SimpleLauncher(use_sim_time=sim_time)
    kwargs = {} if parameters is None else {'parameters': parameters}

    node = sl.node('pkg', 'exe', **kwargs)

    # launch_ros wraps a substitution given as parameters into a ParameterFile
    params = node._Node__parameters
    assert isinstance(params[-1], ParameterFile)
    assert isinstance(params[-1].param_file[0], MergedOverrides)
    if sim_time is not None:
        # dictionary keys are normalized into tuples of TextSubstitution
        names = [key[0].text for p in params if isinstance(p, dict) for key in p]
        assert 'use_sim_time' in names


@pytest.mark.parametrize('parameters, given', [
    (None, []),
    (('a.yaml', 'b.yaml'), ['a.yaml', 'b.yaml']),
    ('a.yaml', ['a.yaml']),
])
def test_node_keeps_each_given_parameter_file(parameters, given):
    node = SimpleLauncher().node('pkg', 'exe', parameters=parameters)

    params = node._Node__parameters
    assert [p.param_file[0].text for p in params[:-1]] == given
    assert isinstance(params[-1].param_file[0], MergedOverrides)


def test_every_node_gets_the_same_file(tmp_path, monkeypatch):
    monkeypatch.setattr(param_overrides.launch_config, '_log_dir', str(tmp_path / 'log'))
    overrides = write(tmp_path, 'o.yaml', {'/**/mode_fsm': section(rate=2.0)})

    without = LaunchContext()
    assert MergedOverrides().perform(without) == MergedOverrides().perform(without)

    given = LaunchContext()
    given.launch_configurations['param_overrides'] = overrides
    path = MergedOverrides().perform(given)
    assert MergedOverrides().perform(given) == path
    with open(path) as f:
        assert yaml.safe_load(f) == {'/**/**/mode_fsm': section(rate=2.0)}
    assert len(list((tmp_path / 'log').iterdir())) == 2


FILE_ARGS = ['param_overrides', 'launch_config']


def declarations(actions, name):
    return [action for action in actions
            if isinstance(action, DeclareLaunchArgument) and action.name == name]


def value_after(actions, context, name):
    # runs the declarations of a description in order, as the launch does; unset reads as empty
    for action in declarations(actions, name):
        action.execute(context)
    return context.launch_configurations.get(name, '')


@pytest.mark.parametrize('name', FILE_ARGS)
@pytest.mark.parametrize('opaque', [False, True])
def test_launcher_declares_the_file_arguments_first_with_no_default(name, opaque):
    sl = SimpleLauncher(use_sim_time=False)
    sl.node('pkg', 'exe')
    if opaque:
        actions = sl.launch_description(opaque_function=lambda: [])().entities
    else:
        actions = sl.launch_description().entities

    declared = declarations(actions, name)
    assert len(declared) == 1
    assert actions.index(declared[0]) < len(FILE_ARGS)
    assert value_after(actions, LaunchContext(), name) == ''


@pytest.mark.parametrize('name', FILE_ARGS)
def test_listed_argument_leaves_the_default_of_an_included_file(name):
    # an application that only includes the file that gives the default
    application = SimpleLauncher().launch_description().entities
    sl = SimpleLauncher(use_sim_time=False)
    sl.declare_arg(name, default_value='site.yaml')
    included = sl.launch_description().entities

    context = LaunchContext()
    assert value_after(application, context, name) == ''
    assert name not in context.launch_configurations
    assert value_after(application + included, LaunchContext(), name) == 'site.yaml'


@pytest.mark.parametrize('name', FILE_ARGS)
def test_given_value_wins_over_the_empty_default(name):
    # a value from the command line or from the including file is set before this file runs
    context = LaunchContext()
    context.launch_configurations[name] = 'site.yaml'

    assert value_after(SimpleLauncher().launch_description().entities, context, name) == 'site.yaml'


@pytest.mark.parametrize('name', FILE_ARGS)
def test_own_declaration_is_kept_and_moved_first(name):
    sl = SimpleLauncher(use_sim_time=False)
    sl.node('pkg', 'exe')
    sl.declare_arg(name, default_value='site.yaml')
    actions = sl.launch_description().entities

    declared = declarations(actions, name)
    assert len(declared) == 1
    assert actions.index(declared[0]) < len(FILE_ARGS)

    # the listed argument of an included file runs later and changes nothing
    included = SimpleLauncher().launch_description().entities
    context = LaunchContext()
    assert value_after(actions + included, context, name) == 'site.yaml'


def test_sections_of_nodes_a_process_creates_are_kept():
    # a Nav2 server creates its costmap as a node of its own process
    merged = {'/**/controller_server': section(rate=2.0),
              '/**/local_costmap': {'local_costmap': section(width=6)}}

    assert node_sections(merged) == {
        '/**/**/controller_server': section(rate=2.0),
        '/**/**/local_costmap': {'local_costmap': section(width=6)}}


@pytest.mark.parametrize('name, namespace, push, expected', [
    ('mode_fsm', 'brain', None, '/brain/mode_fsm'),
    ('mode_fsm', None, 'brain', '/brain/mode_fsm'),
    ('rviz2', None, None, '/rviz2'),
    (None, 'brain', None, None),
])
def test_full_node_name(name, namespace, push, expected):
    context = LaunchContext()
    if push is not None:
        context.launch_configurations['ros_namespace'] = push
    node = Node(package='pkg', executable='exe', name=name, namespace=namespace)
    node._perform_substitutions(context)

    assert full_node_name(node) == expected
