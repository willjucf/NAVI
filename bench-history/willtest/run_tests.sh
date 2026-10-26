#!/usr/bin/env bash
#
# Run every NAVI virtual test suite, both languages, one command.
#
# Nothing here needs to be installed first beyond what the Pi already has:
# python3 with python3-opencv, and g++. There is no pytest dependency and no
# ESP32 has to be attached.
#
#   ./run_tests.sh              everything
#   ./run_tests.sh python       just the Python suites
#   ./run_tests.sh firmware     just the native firmware logic suite
#
# Exits non zero if anything failed, so it can gate a commit or a flash.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WHICH="${1:-all}"
FAILED=0

run_python() {
    echo
    echo "==============================================="
    echo " Python suites  (vision, depth, control, threads)"
    echo "==============================================="
    # Run from tests/ so fixtures.py is importable without packaging the
    # directory. fixtures.py puts the project root on sys.path itself.
    if ( cd "$HERE/tests" && python3 -m unittest discover -s . -p 'test_*.py' -v ); then
        echo "Python suites passed."
    else
        echo "Python suites FAILED."
        FAILED=1
    fi
}

run_firmware() {
    echo
    echo "==============================================="
    echo " Firmware logic suite  (host g++, no ESP32)"
    echo "==============================================="
    if ! command -v g++ >/dev/null 2>&1; then
        echo "g++ not found, skipping. Install it with:"
        echo "  sudo apt install build-essential"
        return
    fi

    # make is the normal path, but it is not installed everywhere the rest of
    # this toolchain is, so fall back to calling the compiler directly. The
    # build is two files, so there is nothing make is really needed for.
    local ok=0
    if command -v make >/dev/null 2>&1; then
        ( cd "$HERE/firmware/test" && make --no-print-directory run ) && ok=1
    else
        echo "(make not found, compiling directly)"
        ( cd "$HERE/firmware/test" &&
          g++ -std=c++17 -Wall -Wextra -O1 -I../main \
              test_navi_logic.cpp ../main/navi_logic.cpp -o test_navi_logic &&
          ./test_navi_logic ) && ok=1
    fi

    if [ "$ok" -eq 1 ]; then
        echo "Firmware suite passed."
    else
        echo "Firmware suite FAILED."
        FAILED=1
    fi

    # The three ESP IDF files cannot be compiled here, so parse them against
    # stub headers. Catches a typo without a flash cycle. It proves nothing
    # about ESP IDF API correctness; see firmware/test/stubs/README.md.
    echo
    echo "ESP IDF sources, syntax only:"
    for f in main.cpp safety.cpp sensors.cpp; do
        if g++ -fsyntax-only -std=c++17 -Wall -Wextra \
               -I "$HERE/firmware/test/stubs" -I "$HERE/firmware/main" \
               "$HERE/firmware/main/$f" 2>/dev/null; then
            echo "  $f  OK"
        else
            echo "  $f  FAILED"
            FAILED=1
        fi
    done
}

case "$WHICH" in
    python)   run_python ;;
    firmware) run_firmware ;;
    all)      run_python; run_firmware ;;
    *)        echo "usage: $0 [all|python|firmware]"; exit 2 ;;
esac

echo
if [ "$FAILED" -eq 0 ]; then
    echo "ALL SUITES PASSED"
else
    echo "SOME SUITES FAILED"
fi
exit "$FAILED"
