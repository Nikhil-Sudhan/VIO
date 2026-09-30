"""Corrupt-input tests for actual sample-loss and timestamp semantics."""
import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from analyse_recording import decode


def word(tag, count, data):
    value = tag*8+count*2
    value |= value.bit_count()%2
    return bytes([value])+data


def slot(tick, count):
    return (word(4,count,struct.pack('<I',tick)+b'\x00\x55')+
            word(1,count,struct.pack('<hhh',-1,2,-3))+
            word(2,count,struct.pack('<hhh',4,-5,8192)))


def batch(payload, status=None):
    count=len(payload)//7
    status=status if status is not None else bytes([count & 255,count>>8])
    return {'kind':'fifo','read_words':count,'raw_hex':payload.hex(),
            'status_hex':status.hex(),'level_words':status[0]|((status[1]&3)<<8),
            'overflow_or_full':bool(status[1]&0x68),'host_after_boot_ns':1000000000}


class DecodeTests(unittest.TestCase):
    def test_signed_values_and_batch_boundary(self):
        payload=slot(100,0)+slot(292,1)
        groups,_,_,faults,trimmed=decode([batch(payload[:14]),batch(payload[14:])])
        self.assertFalse(faults);self.assertFalse(trimmed)
        self.assertEqual(groups[0]['gyro'],(-1,2,-3))
        self.assertEqual(groups[1]['accel'],(4,-5,8192))

    def test_hardware_rollover(self):
        groups,_,_,faults,_=decode([batch(slot(2**32-100,3)+slot(92,0))])
        self.assertFalse(faults)
        self.assertEqual(groups[1]['ticks']-groups[0]['ticks'],192)

    def test_missing_four_slots_not_hidden_by_counter_wrap(self):
        groups,_,_,faults,_=decode([batch(slot(100,0)+slot(1060,1))])
        self.assertEqual(len(groups),2)
        self.assertTrue(any('slot interval' in f for f in faults))

    def test_missing_interior_acceleration(self):
        groups,_,_,faults,_=decode([batch(slot(100,0)[:14]+slot(292,1))])
        self.assertEqual(len(groups),1)
        self.assertIn('Incomplete interior FIFO time slot',faults)

    def test_boundary_trim_is_explicit(self):
        groups,_,_,faults,trimmed=decode([batch(slot(100,0)+slot(292,1)[:14])])
        self.assertFalse(faults);self.assertTrue(trimmed);self.assertEqual(len(groups),1)

    def test_parity_failure(self):
        payload=bytearray(slot(100,0));payload[7]^=1
        self.assertIn('FIFO tag parity error',decode([batch(payload)])[3])

    def test_overflow_status(self):
        self.assertIn('FIFO overflow/full flag',decode([batch(slot(100,0),b'\x03\x60')])[3])

    def test_lying_overflow_flag_rejected(self):
        record=batch(slot(100,0),b'\x03\x60');record['overflow_or_full']=False
        with self.assertRaises(ValueError):decode([record])

    def test_wrong_payload_length_rejected(self):
        record=batch(slot(100,0));record['read_words']=4
        with self.assertRaises(ValueError):decode([record])

    def test_reset_is_not_rollover(self):
        faults=decode([batch(slot(1000,0)+slot(100,1))])[3]
        self.assertIn('Sensor timestamp reversed',faults)


if __name__=='__main__':unittest.main()
