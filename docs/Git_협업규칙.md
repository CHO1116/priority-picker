# 🌿 Git 협업 규칙

> 평가 항목 "문서 및 코드의 엄격한 버전관리" 대응. 1주 프로젝트이므로 **단순하게** 갑니다.

## 저장소 구조 (제안)

```
robo_ws/
├── README.md                  # docs/README.md 내용을 루트로 복사
├── docs/                      # 기획 문서, 회의록, 이슈 로그
├── src/
│   ├── picker_interfaces/     # 커스텀 msg/srv/action      (조영준)
│   ├── picker_vision/         # vision_node                 (이준희)
│   ├── picker_planner/        # order_manager, task_planner (조영준)
│   ├── picker_control/        # robot_controller            (류창표)
│   └── picker_bringup/        # launch 파일, config/*.yaml   (조영준)
└── results/                   # 측정 CSV, 그래프
```

> 패키지를 담당자별로 나누면 서로 같은 파일을 수정할 일이 적어 충돌이 거의 생기지 않습니다.

## 브랜치

- `main`: 항상 실행 가능한 상태. 직접 push 금지
- `feat/<담당영역>-<내용>`: 예) `feat/vision-hsv-detect`, `feat/control-grasp-fail`
- `fix/<내용>`: 버그 수정

작업 → PR 생성 → 팀원 1명 확인(간단히 OK 코멘트라도) → `main`에 merge

## 커밋 메시지

```
<타입>: <무엇을 했는지 한 줄>

feat: 색상별 HSV 임계값 YAML 로딩 추가
fix: 깊이값 0일 때 좌표 계산 오류 수정
docs: Day3 회의록 추가
refactor: 우선순위 정렬 함수 분리
test: 가짜 큐브 퍼블리셔 추가
```

## 태그 (마일스톤)

| 태그 | 시점 |
|---|---|
| `v0.1-single-pick` | Day 3: 인식한 큐브 1개 피킹 성공 |
| `v0.2-full-scenario` | Day 4: 전체 시나리오 동작 |
| `v1.0-freeze` | Day 5: 기능 동결, 발표 버전 |

```bash
git tag v0.1-single-pick
```

```bash
git push origin --tags
```

## 하지 말 것
- `build/`, `install/`, `log/` 폴더 커밋 금지 → `.gitignore`에 추가
- 비밀번호, 개인 정보 커밋 금지
- 하루 넘게 push하지 않고 쌓아두기 금지 (매일 저녁 push)
