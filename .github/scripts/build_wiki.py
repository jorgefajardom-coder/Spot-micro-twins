"""Convierte docs/wiki/ al formato de la pestaña Wiki de GitHub.

La pestaña Wiki es un repositorio git aparte (<repo>.wiki.git) con todas las
páginas en una sola carpeta. Este script:

  * aplana docs/wiki/cerebro/*.md y docs/wiki/brain/*.md en páginas
    Cerebro-NN-*.md y Brain-NN-*.md (los README pasan a Cerebro.md y Brain.md),
  * reescribe los enlaces entre páginas al nombre de página de la wiki,
  * convierte los enlaces a código e imágenes del repo en URLs absolutas a GitHub,
  * genera Home.md (portada bilingüe) y _Sidebar.md (índice lateral).

docs/wiki/ sigue siendo la fuente: no edites las páginas en la pestaña Wiki,
se sobrescriben en cada sincronización.

Uso:
    python .github/scripts/build_wiki.py SALIDA [--repo dueño/repo] [--ref main]
"""

import argparse
import re
import shutil
import subprocess
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[2]
WIKI = ROOT / "docs" / "wiki"
LANGS = {"cerebro": "Cerebro", "brain": "Brain"}
IMAGE_EXT = {".png", ".jpg", ".jpeg", ".gif", ".svg", ".webp"}
LINK = re.compile(r"(!?)\[([^\]]*)\]\(([^)\s]+)\)")
ATTR = re.compile(r'\b(src|href)="([^"]+)"')


def default_repo():
    url = subprocess.check_output(
        ["git", "-C", str(ROOT), "remote", "get-url", "origin"], text=True
    ).strip()
    m = re.search(r"github\.com[:/]([^/]+/[^/]+?)(?:\.git)?$", url)
    if not m:
        raise SystemExit(f"No reconozco el remoto {url!r}; usa --repo dueño/repo")
    return m.group(1)


def page_name(md: Path) -> str:
    """docs/wiki/cerebro/01-conectoma.md -> Cerebro-01-conectoma"""
    prefix = LANGS[md.parent.name]
    return prefix if md.stem == "README" else f"{prefix}-{md.stem}"


def pages():
    for folder in LANGS:
        yield from sorted((WIKI / folder).glob("*.md"))


def rewrite(md: Path, text: str, repo: str, ref: str) -> str:
    def target_url(target, image):
        """Destino nuevo de un enlace relativo, o None si se deja igual."""
        if re.match(r"^[a-z]+:|^#", target):
            return None
        path, _, anchor = target.partition("#")
        resolved = (md.parent / path).resolve()
        try:
            rel = PurePosixPath(resolved.relative_to(ROOT).as_posix())
        except ValueError:
            return None
        anchor = f"#{anchor}" if anchor else ""
        if resolved.suffix == ".md" and resolved.parent.parent == WIKI and resolved.parent.name in LANGS:
            return f"{page_name(resolved)}{anchor}"
        kind = "raw" if image or rel.suffix.lower() in IMAGE_EXT else "blob"
        if resolved.is_dir():
            kind = "tree"
        return f"https://github.com/{repo}/{kind}/{ref}/{rel}{anchor}"

    def fix(m):
        bang, label, target = m.groups()
        url = target_url(target, bool(bang))
        return m.group(0) if url is None else f"{bang}[{label}]({url})"

    def fix_attr(m):
        attr, target = m.groups()
        url = target_url(target, attr == "src")
        return m.group(0) if url is None else f'{attr}="{url}"'

    out, in_code = [], False
    for line in text.splitlines(keepends=True):
        if line.lstrip().startswith("```"):
            in_code = not in_code
        out.append(line if in_code else ATTR.sub(fix_attr, LINK.sub(fix, line)))
    return "".join(out)


def title(md: Path) -> str:
    first = md.read_text(encoding="utf-8").splitlines()[0]
    return first.lstrip("# ").strip()


def sidebar(repo: str) -> str:
    lines = []
    for folder, heading in (("cerebro", "Cerebro (español)"), ("brain", "Brain (English)")):
        lines.append(f"**[{heading}]({LANGS[folder]})**\n")
        for md in sorted((WIKI / folder).glob("[0-9]*.md")):
            lines.append(f"- [{title(md)}]({page_name(md)})")
        lines.append("")
    lines.append(f"---\n[Repositorio](https://github.com/{repo})")
    return "\n".join(lines) + "\n"


def home(repo: str) -> str:
    raw = f"https://github.com/{repo}/raw/main/docs/assets"
    return f"""# Spot Micro Twins · Wiki

<p align="center">
  <img src="{raw}/spot_brain.gif" alt="El Spot explorando la cueva con el cerebro de la mosca" width="720">
</p>

**Español** · El cerebro del Spot Micro: el conectoma completo de una mosca de la
fruta (138.639 neuronas) pilotando el robot en la cueva.
→ **[Entrar a la wiki en español](Cerebro)**

**English** · The Spot Micro's brain: a fruit fly's full connectome (138,639
neurons) driving the robot through the cave.
→ **[Open the wiki in English](Brain)**

"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path, help="carpeta de salida (p. ej. el clon de <repo>.wiki.git)")
    ap.add_argument("--repo", help="dueño/repo en GitHub (por defecto, el remoto origin)")
    ap.add_argument("--ref", default="main", help="rama a la que apuntan los enlaces al código")
    args = ap.parse_args()
    repo = args.repo or default_repo()
    out = args.out
    out.mkdir(parents=True, exist_ok=True)

    # Borra las páginas generadas antes (no toca .git ni otros archivos).
    for old in out.glob("*.md"):
        old.unlink()

    for md in pages():
        text = rewrite(md, md.read_text(encoding="utf-8"), repo, args.ref)
        (out / f"{page_name(md)}.md").write_text(text, encoding="utf-8")
    (out / "Home.md").write_text(home(repo), encoding="utf-8")
    (out / "_Sidebar.md").write_text(sidebar(repo), encoding="utf-8")
    print(f"{len(list(out.glob('*.md')))} páginas en {out}")


if __name__ == "__main__":
    main()
