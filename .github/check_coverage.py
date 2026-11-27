"""Enforce the package line-coverage target on a CoverageTools LCOV report."""

import sys
from pathlib import Path


def check(report):
    expected = set(Path("src").rglob("*.jl"))
    reported = set()
    covered = total = 0
    for record in report.read_text().split("end_of_record"):
        if not record.strip():
            continue
        fields = dict(line.split(":", 1) for line in record.splitlines() if line)
        path = Path(fields["SF"])
        if path in reported:
            raise ValueError(f"duplicate coverage record for {path}")
        reported.add(path)
        hits, lines = int(fields["LH"]), int(fields["LF"])
        if not 0 <= hits <= lines:
            raise ValueError(f"invalid line counts for {path}")
        covered += hits
        total += lines
    if reported != expected:
        raise ValueError(f"LCOV source files differ from the package: {reported ^ expected}")
    if not total:
        raise ValueError("LCOV report contains no executable lines")
    print(f"Package line coverage: {covered}/{total} ({100 * covered / total:.3f}%)")
    if 10 * covered <= 9 * total:
        raise SystemExit("Package line coverage must exceed 90%")


if __name__ == "__main__":
    check(Path(sys.argv[1]))
