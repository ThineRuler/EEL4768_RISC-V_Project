"""Splice a config `.data` block into a RISC-V source file.

The rule (documented in configs/README.md):
  replace every line from the first line whose stripped text begins with ".data"
  up to, but NOT including, the first line at-or-after it whose stripped text
  begins with ".text".
"""


def _find(lines, directive, start=0):
    for i in range(start, len(lines)):
        if lines[i].strip().startswith(directive):
            return i
    return -1


def splice(src_text, data_block_text):
    """Return src_text with its .data section replaced by data_block_text.

    Raises ValueError if the source has no `.data`, no `.text`, or `.text`
    before `.data`.
    """
    lines = src_text.splitlines(keepends=True)
    d = _find(lines, ".data")
    if d < 0:
        raise ValueError("no line beginning with .data")
    t = _find(lines, ".text", d)
    if t < 0:
        if _find(lines, ".text") >= 0:
            raise ValueError(".text appears before .data")
        raise ValueError("no line beginning with .text")
    block = data_block_text
    if block and not block.endswith("\n"):
        block += "\n"
    return "".join(lines[:d]) + block + "".join(lines[t:])
