# ESP IDF stub headers

Just enough fake ESP IDF to run `g++ -fsyntax-only` over `main.cpp`,
`safety.cpp` and `sensors.cpp` on a machine with no ESP IDF installed.

## What this proves

- The files parse. No missing brace, no missing semicolon, no stray token.
- Every function, variable and type the code refers to is actually declared.
- Calls into our own code have the right argument counts and types, so a
  renamed function or a changed signature is caught here.
- The namespace and include structure hangs together.

## What this does not prove

- **Nothing about whether the ESP IDF API is used correctly.** These stubs are
  written to match how the code calls the real API, so if a real function takes
  its arguments in a different order, or a struct field has a different name,
  the stub is wrong in the same way the code is and both agree. Only a real
  `idf.py build` catches that.
- Nothing about runtime behaviour, timing, or whether the thing works.

So a passing syntax check means the code is worth flashing, not that it is
correct. The real build is still the gate. This exists because the alternative
on a machine with no toolchain is no checking at all, and a typo found here is
a flash cycle saved.

The logic that genuinely matters is deliberately not in these files. It lives in
`navi_logic.cpp`, which has no ESP IDF dependency and is compiled and tested for
real by `test_navi_logic.cpp`.
