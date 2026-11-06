'''
Unit tests of the param_overrides merge and of its place in SimpleLauncher.node()
Run with: python3 -m pytest test/test_param_overrides.py
'''

import pytest
import yaml
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


def test_no_override_gives_a_file_that_sets_nothing():
    assert node_sections({}) == {'/**/**': section()}


def test_unnamed_node_gets_sections_no_other_file_uses_wildcard_first():
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


def overrides_file(context, name):
    node = Node(package='pkg', executable='exe', name=name)
    node._perform_substitutions(context)
    overrides = MergedOverrides()
    overrides.node = node
    return overrides.perform(context)


def test_nodes_with_nothing_to_override_share_one_file(tmp_path, monkeypatch):
    monkeypatch.setattr(param_overrides.launch_config, '_log_dir', str(tmp_path / 'log'))
    overrides = write(tmp_path, 'o.yaml', {'/**/mode_fsm': section(rate=2.0)})

    without = LaunchContext()
    assert overrides_file(without, 'a') == overrides_file(without, 'b')

    given = LaunchContext()
    given.launch_configurations['param_overrides'] = overrides
    assert overrides_file(given, 'a') == overrides_file(given, 'b')
    assert overrides_file(given, 'mode_fsm') != overrides_file(given, 'a')
    assert len(list((tmp_path / 'log').iterdir())) == 3


def test_launcher_does_not_declare_the_argument():
    # an included file must not hide the default given by the file that includes it
    actions = SimpleLauncher(use_sim_time=False).launch_description().entities

    assert not [action for action in actions
                if isinstance(action, DeclareLaunchArgument) and action.name == 'param_overrides']


def test_named_node_gets_its_sections_under_its_full_name():
    merged = {'/**': section(shared=1, rate=1.0),
              '/**/mode_fsm': section(rate=2.0),
              '/**/other': section(x=1)}

    assert node_sections(merged, '/brain/mode_fsm') == {
        '/brain/mode_fsm': section(shared=1, rate=2.0)}


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
