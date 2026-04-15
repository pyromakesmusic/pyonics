class DummyController:
    def __init__(self):
        self.pressures = None

    def set_pressures(self, *args):
        print("RAW ARGS:", args)
        sliced = list(args)
        print("SLICED:", sliced)
        self.pressures = sliced


def test_set_pressures_behavior():
    ctrl = DummyController()

    # Simulate OSC-style call
    ctrl.set_pressures("/pressures", 1.0, 2.0, 3.0)

    print("FINAL PRESSURES:", ctrl.pressures)
    assert ctrl.pressures == ['/pressures', 1.0, 2.0, 3.0]

test_set_pressures_behavior()