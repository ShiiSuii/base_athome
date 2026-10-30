"""Tests offline para el esquema de un solo dueño por joint en /joint_states (HOME-20).

Verifica:
1. Que view_robot.launch.py declare source_list con los tópicos:
   ['/arm/joint_states', '/pan_tilt/joint_states', '/torso/joint_states']
2. Que manipulacion_autonoma.py _publish_joint_states NO publique joint1..joint8 en /joint_states,
   sino torso_lift_joint en /torso/joint_states y pose comandada en /arm/joint_states_cmd.
3. Que hri_receptionist_challenge.py _publish_joint_states NO publique joint1..joint8 en /joint_states,
   sino torso_lift_joint en /torso/joint_states y pose comandada en /arm/joint_states_cmd.
4. Que _ensure_robot_state_publisher no duplique procesos y pase el URDF como parámetro robot_description.

Ejecución:
    ROS_DOMAIN_ID=77 ~/ros2_vision_venv/bin/python -m pytest -q src/base_athome_perception/test/test_joint_states_publishers.py
"""
import ast
import importlib.util
import os
from unittest import mock

import pytest

HERE = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
LAUNCH_PATH = os.path.join(REPO_ROOT, 'src', 'hoverboard_base', 'launch', 'view_robot.launch.py')
HRI_SCRIPT = os.path.join(HERE, '..', 'scripts', 'hri_receptionist_challenge.py')
MANIP_SCRIPT = os.path.join(HERE, '..', 'scripts', 'manipulacion_autonoma.py')


# -----------------------------------------------------------------------------
# Fixtures para importar los módulos de percepción con mocks si es necesario
# -----------------------------------------------------------------------------

@pytest.fixture(scope='module')
def hri_module():
    spec = importlib.util.spec_from_file_location('hri_receptionist_challenge', HRI_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:
        pytest.skip(f'No se pudo importar hri_receptionist_challenge: {exc}')
    return mod


@pytest.fixture(scope='module')
def manip_module():
    spec = importlib.util.spec_from_file_location('manipulacion_autonoma', MANIP_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:
        pytest.skip(f'No se pudo importar manipulacion_autonoma: {exc}')
    return mod


# -----------------------------------------------------------------------------
# Test 1: view_robot.launch.py declara source_list con los 3 tópicos
# -----------------------------------------------------------------------------

def test_view_robot_launch_source_list_in_ast():
    """Valida mediante análisis AST estático que source_list contenga los 3 tópicos requeridos."""
    assert os.path.exists(LAUNCH_PATH), f"No existe el archivo de launch: {LAUNCH_PATH}"
    with open(LAUNCH_PATH, 'r', encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=LAUNCH_PATH)

    found_source_list = False
    expected_topics = ['/arm/joint_states', '/pan_tilt/joint_states', '/torso/joint_states']

    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, val in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == 'source_list':
                    if isinstance(val, (ast.List, ast.Tuple)):
                        actual_topics = [
                            elt.value for elt in val.elts
                            if isinstance(elt, ast.Constant)
                        ]
                        assert actual_topics == expected_topics, (
                            f"source_list en launch no coincide: esperado {expected_topics}, obtenido {actual_topics}"
                        )
                        found_source_list = True

    assert found_source_list, "No se encontró el parámetro 'source_list' en view_robot.launch.py"


def test_view_robot_launch_generate_launch_description():
    """Valida vía ejecución de generate_launch_description() que joint_state_publisher configure source_list."""
    try:
        from launch import LaunchContext
        from launch_ros.actions import Node
        from launch_ros.utilities import evaluate_parameters
    except ImportError as e:
        pytest.skip(f"launch / launch_ros no disponible: {e}")

    spec = importlib.util.spec_from_file_location('view_robot', LAUNCH_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()

    jsp_nodes = [
        e for e in ld.entities
        if isinstance(e, Node) and getattr(e, 'node_executable', None) == 'joint_state_publisher'
    ]
    assert len(jsp_nodes) >= 1, "No se encontró nodo joint_state_publisher en LaunchDescription"
    jsp = jsp_nodes[0]

    ctx = LaunchContext()
    ctx.launch_configurations['use_sim_time'] = 'false'
    evaluated = evaluate_parameters(ctx, jsp._Node__parameters)

    source_list = None
    for param_dict in evaluated:
        if isinstance(param_dict, dict) and 'source_list' in param_dict:
            source_list = list(param_dict['source_list'])
            break

    expected = ['/arm/joint_states', '/pan_tilt/joint_states', '/torso/joint_states']
    assert source_list == expected, f"source_list evaluado incorrecto: {source_list}"


# -----------------------------------------------------------------------------
# Test 2: manipulacion_autonoma._publish_joint_states
# -----------------------------------------------------------------------------

def test_manipulacion_autonoma_publish_joint_states_ownership(manip_module):
    """Verifica que manipulacion_autonoma no publique joint1..joint8 en /joint_states."""
    cls = getattr(manip_module, 'ManipulacionAutonomaNodo', None)
    if cls is None:
        cls = next(v for v in vars(manip_module).values() if isinstance(v, type) and hasattr(v, '_publish_joint_states'))

    n = cls.__new__(cls)
    mock_clock = mock.Mock()
    mock_now = mock.Mock()
    mock_now.to_msg.return_value = mock.Mock()
    mock_clock.now.return_value = mock_now
    n.get_clock = lambda: mock_clock

    mock_torso_pub = mock.Mock()
    mock_arm_cmd_pub = mock.Mock()
    mock_legacy_joint_pub = mock.Mock()

    n.pub_torso_joint_states = mock_torso_pub
    n.pub_arm_joint_states_cmd = mock_arm_cmd_pub
    n.pub_joint_states = mock_legacy_joint_pub

    n.arm_joints_deg = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    n.gripper_opening_mm = 50.0
    n.pose_pre_grasp = [0.0] * 6

    n._publish_joint_states()

    # /joint_states NO debe haber sido llamado
    assert not mock_legacy_joint_pub.publish.called, "NUNCA debe publicarse en /joint_states desde manipulacion_autonoma"

    # /torso/joint_states debe contener torso_lift_joint
    assert mock_torso_pub.publish.called, "Debe publicarse en /torso/joint_states"
    torso_msg = mock_torso_pub.publish.call_args[0][0]
    assert torso_msg.name == ["torso_lift_joint"]
    assert len(torso_msg.position) == 1
    for blocked_joint in [f"joint{i}" for i in range(1, 9)]:
        assert blocked_joint not in torso_msg.name

    # /arm/joint_states_cmd debe contener joint1..joint8 (comandados)
    assert mock_arm_cmd_pub.publish.called, "Debe publicarse en /arm/joint_states_cmd"
    arm_msg = mock_arm_cmd_pub.publish.call_args[0][0]
    for j in [f"joint{i}" for i in range(1, 9)]:
        assert j in arm_msg.name


# -----------------------------------------------------------------------------
# Test 3: hri_receptionist_challenge._publish_joint_states
# -----------------------------------------------------------------------------

def test_hri_receptionist_publish_joint_states_ownership(hri_module):
    """Verifica que hri_receptionist_challenge no publique joint1..joint8 en /joint_states."""
    cls = getattr(hri_module, 'HRIReceptionistChallenge', None)
    if cls is None:
        cls = next(v for v in vars(hri_module).values() if isinstance(v, type) and hasattr(v, '_publish_joint_states'))

    n = cls.__new__(cls)
    mock_clock = mock.Mock()
    mock_now = mock.Mock()
    mock_now.to_msg.return_value = mock.Mock()
    mock_clock.now.return_value = mock_now
    n.get_clock = lambda: mock_clock

    mock_torso_pub = mock.Mock()
    mock_arm_cmd_pub = mock.Mock()
    mock_legacy_joint_pub = mock.Mock()

    n.pub_torso_joint_states = mock_torso_pub
    n.pub_arm_joint_states_cmd = mock_arm_cmd_pub
    n.pub_joint_states = mock_legacy_joint_pub

    n.arm_joints_deg = [0.0, 15.0, 30.0, 0.0, 45.0, 0.0]
    n.target_pan_deg = 10.0
    n.target_tilt_deg = -5.0

    n._publish_joint_states()

    # /joint_states NO debe haber sido llamado
    assert not mock_legacy_joint_pub.publish.called, "NUNCA debe publicarse en /joint_states desde hri_receptionist_challenge"

    # /torso/joint_states debe contener torso_lift_joint
    assert mock_torso_pub.publish.called, "Debe publicarse en /torso/joint_states"
    torso_msg = mock_torso_pub.publish.call_args[0][0]
    assert torso_msg.name == ["torso_lift_joint"]
    for blocked_joint in [f"joint{i}" for i in range(1, 9)]:
        assert blocked_joint not in torso_msg.name

    # /arm/joint_states_cmd debe contener joint1..joint6
    assert mock_arm_cmd_pub.publish.called, "Debe publicarse en /arm/joint_states_cmd"
    arm_msg = mock_arm_cmd_pub.publish.call_args[0][0]
    for j in [f"joint{i}" for i in range(1, 7)]:
        assert j in arm_msg.name


# -----------------------------------------------------------------------------
# Test 4: _ensure_robot_state_publisher en manipulacion_autonoma
# -----------------------------------------------------------------------------

def test_ensure_robot_state_publisher_skips_if_already_running(manip_module):
    """Si pgrep encuentra robot_state_publisher, no debe lanzar un segundo proceso."""
    cls = next(v for v in vars(manip_module).values() if isinstance(v, type) and hasattr(v, '_ensure_robot_state_publisher'))
    n = cls.__new__(cls)
    mock_logger = mock.Mock()
    n.get_logger = lambda: mock_logger
    n._rsp_proc = None

    with mock.patch('subprocess.run') as mock_run, mock.patch('subprocess.Popen') as mock_popen:
        mock_run.return_value = mock.Mock(returncode=0, stdout='1234\n')
        n._ensure_robot_state_publisher()
        mock_popen.assert_not_called()
        assert n._rsp_proc is None


def test_ensure_robot_state_publisher_launches_with_param(manip_module):
    """Si robot_state_publisher no corre, debe lanzarse con -p robot_description:=<urdf>."""
    cls = next(v for v in vars(manip_module).values() if isinstance(v, type) and hasattr(v, '_ensure_robot_state_publisher'))
    n = cls.__new__(cls)
    mock_logger = mock.Mock()
    n.get_logger = lambda: mock_logger
    n._rsp_proc = None

    fake_urdf = "<robot name='base_athome'/>"

    with mock.patch('subprocess.run') as mock_run, \
         mock.patch('subprocess.Popen') as mock_popen, \
         mock.patch('os.path.exists', return_value=True), \
         mock.patch('builtins.open', mock.mock_open(read_data=fake_urdf)):

        mock_run.return_value = mock.Mock(returncode=1, stdout='')
        n._ensure_robot_state_publisher()

        mock_popen.assert_called_once()
        cmd = mock_popen.call_args[0][0]

        # Verificar que use ros2 run robot_state_publisher robot_state_publisher
        assert cmd[:4] == ["ros2", "run", "robot_state_publisher", "robot_state_publisher"]

        # Verificar que use el parámetro robot_description y NO una ruta posicional
        assert "-p" in cmd or "--ros-args" in cmd
        param_arg = [arg for arg in cmd if arg.startswith("robot_description:=")]
        assert len(param_arg) == 1, f"No se encontró el argumento robot_description:= en {cmd}"
        assert fake_urdf in param_arg[0]
