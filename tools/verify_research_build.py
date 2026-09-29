"""Verify application CAN driver objects contain no hardware TX/normal-mode calls.
Usage: python tools/verify_research_build.py --nm /path/to/xtensa-esp32s3-elf-nm
This is a binary check of application references, not electrical certification.
"""
import argparse
import pathlib
import subprocess

root = pathlib.Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--nm', required=True)
args = parser.parse_args()
for name in ('waveshare-s3-can', 'bench', 'research'):
    folder = root / 'esp32/.pio/build' / name / 'src'
    objects = list(folder.glob('*.o'))
    assert objects, f'{name}: build first'
    symbols = subprocess.check_output([args.nm, '-u', *map(str, objects)], text=True)
    for forbidden in ('twai_transmit', 'sendMessage', 'setNormalMode'):
        assert forbidden not in symbols, f'{name}: forbidden reference {forbidden}'
    if name == 'bench':
        assert 'twai_' not in symbols, 'bench must not link a physical TWAI controller'
    assert (folder.parent / 'firmware-merged.bin').is_file()
    print(f'{name}: no physical CAN TX/normal-mode calls in {len(objects)} application objects; merged image exists')
