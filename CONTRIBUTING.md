# 기여 규칙 (브랜치·커밋)

## Find Door 단계별 작업 기록

2026-11-02 사용자 요청에 따라 이 분기에서는 작업 단위마다 한국어 Conventional Commit을
작성하고, 검증된 단계가 완성될 때 작업 브랜치를 로컬 `dev`에 병합합니다.
단계 완료 버전은 `dev`의 `v0.2.0-door-probe.1` 같은 **사전 릴리스 태그**로 남깁니다.
이 로컬 단계 병합·태그는 아래 기존 프로젝트의 PR/`main` 릴리스 절차에 대한 명시적 예외입니다.
검증 실패·미완료 기능을 단계 완료로 표시하지 않고, 원격 push·PR은 사용자 승인 범위에서만 수행합니다.
최종 정식 릴리스와 기반 프로젝트의 `v0.1.0` 이력은 구분합니다.

개인 프로젝트지만, 나중에 히스토리만 보고도 "언제 무엇을 왜 바꿨는지" 알 수 있도록 아래 규칙을 따릅니다.

## 1. 브랜치 전략 — `main` / `dev` + 작업 브랜치

```text
hotfix/…                                  ●──●
                                         /    \
main     ●──────────────────────●───────●──────●──────────▶  릴리스만 (태그)
          \                    ↗ v0.1.0         \ v0.1.1
dev        ●──●──────●──────●─●──────────────────●────────▶  통합 (다음 릴리스 준비)
               \    ↗ \    ↗                  (hotfix를 dev에도 반영)
          feat/…●──●   ●──● fix/…
```

| 브랜치 | 역할 | 분기 원본 | 병합 대상 | 병합 방식 |
|---|---|---|---|---|
| `main` | 단계 완료된 **릴리스 상태**만. 릴리스 태그(`vX.Y.Z`)는 여기에만 | — | — | — |
| `dev` | 작업 통합. 항상 빌드 통과 상태 | `main` | `main` | PR, **Merge commit** |
| `feat/`·`fix/`·`docs/`·`refactor/`·`chore/`·`exp/` | 개별 작업 | `dev` | `dev` | PR, **Squash merge** |
| `hotfix/` | `main`(릴리스)의 긴급 버그 수정 | `main` | `main` → 이후 `dev`에도 반영 | PR, Squash merge |

규칙:

- `main`·`dev`에는 **직접 커밋하지 않고** PR로만 병합합니다.
  - 예외: 저장소 최초 커밋(문서 뼈대)은 `main`에 직접 커밋한 뒤 `dev`를 만듭니다.
- **`dev` 기준**: 빌드·실행 가능. ROS 패키지가 생기기 전(Phase 0 초기)에는 "문서 링크가 깨지지 않음"을 기준으로 합니다.
- **`dev` → `main`**: [로드맵](docs/roadmap.md) 단계 완료(또는 패치 릴리스) 시에만 PR을 열고, 병합 후 `main`에 태그를 답니다. 이 PR은 Squash하지 않고 **Merge commit**으로 병합해 `dev` 히스토리를 보존합니다.
- **`hotfix/`**: `main`에 병합·태그(patch) 후, 같은 수정을 `dev`에도 PR로 반영합니다 (`main` → `dev` 병합 또는 cherry-pick).
- 작업 브랜치는 짧게 유지하고(가능하면 며칠 안에 병합), 병합 후 삭제합니다.
- 작업 시작 전 `dev`를 최신으로 받습니다: `git switch dev && git pull`.

GitHub 저장소 설정 (원격 생성 후):

- Default branch: `dev` (PR 기본 대상이 `dev`가 되도록)
- Branch protection: `main`, `dev` — PR 필수, force push·삭제 금지
- Merge 방식: **Squash merge**(작업 PR용)와 **Merge commit**(릴리스 PR용)을 둘 다 허용

### 브랜치 이름

`<type>/<짧은-설명>` — 소문자, 단어는 `-`로 연결, 영문 권장.

| 접두사 | 용도 | 예시 |
|---|---|---|
| `feat/` | 새 기능 | `feat/indoor-world`, `feat/slam-toolbox-launch` |
| `fix/` | 버그 수정 | `fix/tf-camera-frame` |
| `exp/` | 실험·비교 (병합 안 할 수도 있음) | `exp/cartographer-vs-slam-toolbox` |
| `docs/` | 문서·ADR | `docs/adr-0004-slam-algorithm` |
| `refactor/` | 동작 변화 없는 구조 개선 | `refactor/semantic-node-split` |
| `chore/` | 빌드·설정·CI | `chore/gitignore-ros2` |
| `hotfix/` | 릴리스(`main`) 긴급 수정 | `hotfix/map-save-crash` |

> `exp/` 브랜치는 병합하지 않아도 됩니다. 단, 브랜치를 삭제하면 그 커밋은 새 clone에서 사라질 수 있으므로, 삭제 전에 **보관 태그** `archive/exp-<설명>`을 달아 push하고 [evaluation.md](docs/evaluation.md)·ADR에 그 태그를 기록하세요.

## 2. 커밋 메시지 — Conventional Commits

```text
<type>(<scope>): <요약>

<본문: 무엇을, 왜 (선택)>

<꼬리말: Refs #이슈, BREAKING CHANGE 등 (선택)>
```

- **요약**: 50자 이내, 한국어 가능, 마침표 없음, "~한다/~추가" 같은 명령·완결형
- **본문**: 72자 줄바꿈. "어떻게"보다 "왜"를 씁니다
- 한 커밋에는 한 가지 논리적 변경만

### type

| type | 의미 |
|---|---|
| `feat` | 기능 추가 |
| `fix` | 버그 수정 |
| `docs` | 문서만 변경 |
| `refactor` | 동작 변화 없는 코드 구조 변경 |
| `test` | 테스트 추가·수정 |
| `exp` | 실험용 설정·스크립트 |
| `perf` | 성능 개선 |
| `chore` | 빌드·의존성·설정 |
| `release` | `dev` → `main` 릴리스 병합 커밋 전용 (예: `release: v0.1.0 (Phase 0 완료)`) |

### scope

| scope | 대상 |
|---|---|
| `gazebo` | `mapping_gazebo` (world, launch) |
| `slam` | `mapping_slam` |
| `semantic` | `mapping_semantic` |
| `eval` | `mapping_eval` |
| `adr` | `docs/adr/` |
| `docs` | 그 외 문서 |
| `repo` | 저장소 설정 (.gitignore 등) |

### 예시

```text
feat(gazebo): 복도·방 3개가 있는 실내 매핑 world 추가

도면 치수를 알고 있는 world가 있어야 Phase 1에서 지도 오차를
정량적으로 측정할 수 있다.

Refs #3
```

```text
docs(adr): ADR-0004 SLAM 알고리즘으로 slam_toolbox 선택
```

```text
fix(semantic): 카메라 광학 프레임 회전 누락으로 객체가 90° 틀어지던 문제 수정
```

커밋 템플릿 사용: `git config commit.template .gitmessage`

## 3. Pull Request

- 대상(base) 브랜치: 작업 브랜치 → `dev`, 릴리스 → `main` (`hotfix/`만 `main` 직접)
- 제목은 커밋 규칙과 동일. 릴리스 PR 제목은 `release: v0.1.0 (Phase 0 완료)` 형식
- GitHub 저장소 설정 → Pull Requests → Squash merge 기본 메시지를 **"Pull request title (and description)"**으로 설정합니다. (기본값에서는 커밋이 1개인 PR이 커밋 메시지를 그대로 쓰므로) 병합 직전에 최종 메시지를 한 번 확인합니다
- [PR 템플릿](.github/pull_request_template.md)의 체크리스트를 채웁니다
- 실험 결과가 있으면 수치·지도 캡처를 PR 본문에 첨부

## 4. 태그·릴리스

[로드맵](docs/roadmap.md) 단계 완료 시 `dev` → `main` 병합 후 `main`에 태그를 답니다 (Semantic Versioning). `hotfix/`는 patch 버전을 올립니다.

```bash
git switch main && git pull
git tag -a v0.1.0 -m "Phase 0 완료"
git push origin v0.1.0
```

- **공개 인터페이스** = 토픽 이름·메시지 타입, launch 인자, 저장 지도·객체 지도 파일 형식
- `v1.0.0` 이후: 공개 인터페이스 호환성을 깨면 major, 호환되는 기능 추가는 minor, 버그 수정은 patch
- 아래 표는 **예정값**이며, 위 기준과 충돌하면 기준을 따릅니다

| 태그 (예정) | 시점 |
|---|---|
| `v0.1.0` | Phase 0 완료 |
| `v0.2.0` | Phase 1 완료 |
| `v1.0.0` | Phase 2 완료 (MVP) |
| `v1.x.0` | Phase 3·4 |

## 5. 저장소에 올리지 않는 것

- `build/`, `install/`, `log/` (colcon 산출물)
- 대용량 rosbag (`*.db3`, `*.mcap`) — 필요하면 외부 저장소 링크만 기록
- 연구과제 비공개 자료 — 공개 범위 확인 전에는 커밋하지 않음 ([README](README.md#연구과제와의-경계))
