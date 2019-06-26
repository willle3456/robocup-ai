#!/usr/bin/env python

import unittest

class TestSimSender(unittest.TestCase)
    def test_connect_success(self):
        pass

    def test_connect_fail(self):
        pass

    def test_send_success(self):
        pass

    def test_send_fail(self):
        pass

class TestPlayer(unittest.TestCase)
    def test_update_position(self):
        pass

    def test_update_strat(self):
        pass

    def test_write_output(self):
        pass

    def test_run_init(self):
        pass

    def test_run_action(self):
        pass

    #TODO sim vs action is more for the node level of things

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('player', 'test_sim_sender', TestSimSender)
    rosunit.unitrun('player', 'test_player', TestPlayer)
