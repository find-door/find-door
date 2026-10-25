#!/usr/bin/env python3
"""Markdown의 상대 파일 링크와 제목 앵커 검사 (외부 URL/참조형 링크 제외)."""
import argparse
import html
from pathlib import Path
import re
import unicodedata
from urllib.parse import unquote, urlsplit

SKIP = {'.git', '.agents', '.codex', 'build', 'install', 'log', '__pycache__', '.pytest_cache'}
LINK = re.compile(r'!?\[[^\]\n]*\]\(\s*(?:<([^>]+)>|([^\s()]*(?:\([^()]*\)[^\s()]*)*))(?:\s+[\"\'][^\n]*?[\"\'])?\s*\)')


def prose(text):
    """줄 번호를 보존하며 fenced code와 HTML 주석을 제거한다."""
    text = re.sub(r'<!--.*?-->', lambda m: '\n' * m[0].count('\n'), text, flags=re.S)
    fence, result = None, []
    for line in text.splitlines():
        match = re.match(r'^\s{0,3}(`{3,}|~{3,})', line)
        if fence:
            if match and match[1][0] == fence[0] and len(match[1]) >= len(fence):
                fence = None
            result.append('')
        elif match:
            fence = match[1]
            result.append('')
        else:
            result.append(line)
    return '\n'.join(result)


def anchors(text):
    clean = prose(text)
    found = set(re.findall(r'<[^>]+\b(?:id|name)=[\"\']([^\"\']+)[\"\']', clean))
    used = set()
    lines = clean.splitlines()
    for index, line in enumerate(lines):
        match = re.match(r'^ {0,3}#{1,6}\s+(.+?)(?:\s+#+\s*)?$', line)
        title = match[1] if match else None
        if title is None and index + 1 < len(lines) and line.strip() and re.fullmatch(r' {0,3}(?:=+|-+)\s*', lines[index + 1]):
            title = line.strip()
        if title is None:
            continue
        title = re.sub(r'!?\[([^\]]*)\]\([^)]*\)', r'\1', title)
        title = html.unescape(re.sub(r'<[^>]*>', '', title)).lower()
        slug = ''.join(c for c in title if c in '_-' or not unicodedata.category(c).startswith(('P', 'S'))).replace(' ', '-')
        candidate, suffix = slug, 0
        while candidate in used:
            suffix += 1
            candidate = f'{slug}-{suffix}'
        used.add(candidate)
        found.add(candidate)
    return found


def check(root):
    errors = []
    files = sorted(p for p in root.rglob('*.md') if not any(part in SKIP or part.startswith('.') for part in p.relative_to(root).parts[:-1]) and p.name != 'CLAUDE.md')
    cache = {}
    for source in files:
        clean = prose(source.read_text(encoding='utf-8'))
        clean = re.sub(r'(`+)([^`]|(?!\1)`)*?\1',
                       lambda m: '\n' * m[0].count('\n'), clean)
        for match in LINK.finditer(clean):
            raw = match[1] if match[1] is not None else match[2]
            parsed = urlsplit(raw)
            if parsed.scheme or parsed.netloc or raw.startswith('/'):
                continue
            target = (source.parent / unquote(parsed.path)).resolve() if parsed.path else source
            prefix = f'{source.relative_to(root)}:{clean[:match.start()].count(chr(10)) + 1}: {raw}'
            if not target.exists():
                errors.append(f'{prefix}: 파일 없음')
            elif parsed.fragment and target.suffix.lower() == '.md':
                if target not in cache:
                    cache[target] = anchors(target.read_text(encoding='utf-8'))
                if unquote(parsed.fragment) not in cache[target]:
                    errors.append(f'{prefix}: 앵커 없음')
    return files, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    root = parser.parse_args().root.resolve()
    if not root.is_dir():
        parser.error('--root는 존재하는 디렉터리여야 합니다')
    files, errors = check(root)
    for error in errors:
        print(error)
    print(f'Markdown {len(files)}개 검사, 오류 {len(errors)}개 (외부 URL·참조형 링크 제외)')
    return bool(errors)


if __name__ == '__main__':
    raise SystemExit(main())
