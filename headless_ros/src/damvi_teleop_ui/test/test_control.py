import math
import pytest

from damvi_teleop_ui.control import Controller


def test_stop_and_manual():
    c = Controller()
    c.receive((3., 0., 0., 0., 0., .2), 0.)
    assert c.output(0.)[0] == (0.,) * 6
    c.set_mode('manual')
    c.keys.update(('up', 'left'))
    assert c.output(1.)[0][0] == 2.
    assert c.output(1.)[0][5] == pytest.approx(.35 / .52)
    c.keys.update(('down', 'right'))
    assert c.output(1.)[0] == (0.,) * 6
    c.keys = {'down', 'right'}
    c.speed = 3.25
    assert c.output(1.)[0][0] == -3.25
    assert c.output(1.)[0][5] < 0
    c.set_mode('stop')
    assert c.output(1.)[0] == (0.,) * 6
    assert not c.keys


def test_autonomous_passthrough_timeout_and_reentry():
    c = Controller()
    values = (1.2, .1, .2, .3, .4, -.25)
    c.receive(values, 0.)
    c.set_mode('auto')
    assert c.output(0.)[0] == (0.,) * 6
    c.receive(values, 10.)
    assert c.output(10.1)[0] == values
    assert c.output(10.6)[0] == (0.,) * 6
    c.set_mode('manual')
    c.set_mode('auto')
    assert c.output(10.7)[0] == (0.,) * 6
    c.timeout = 0.
    c.receive(values, 11.)
    assert c.output(10000.)[0] == values
    c.receive((math.nan,) * 6, 10001.)
    assert c.output(10001.)[0] == (0.,) * 6


@pytest.mark.parametrize('kwargs', [dict(speed=-1), dict(max_steering=0),
                                   dict(timeout=-1), dict(speed=math.inf)])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        Controller(**kwargs)
