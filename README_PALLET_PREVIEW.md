# 학습 모델 실시간 추정값 화면

실행 파일: `scripts/preview_pallet_pose.py`.
기본 모델은 `/home/sojeong/turtlebot3_ws/src/best.pt`이며 기존
`config/pallet_alignment_yolo.yaml`의 카메라·치수 설정을 읽는다.
ROS JPEG, CameraInfo와 TF를 구독한다. 속도 발행·이동 액션·실행 스위치는 없다.
미리보기는 IPPE의 두 후보 각각을 LM으로 추가 최적화한 뒤 기하 조건을 검사한다.
로봇 제어용 정렬기의 알고리즘과 오차 제한은 변경하지 않는다.

## 실행 (개발 PC의 그래픽 터미널)

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/scripts/preview_pallet_pose.py
```

카메라는 미리 실행되어 있어야 한다. 이 코드는 카메라를 켜거나 끄지 않는다.
별도 pip 설치 없이 기존 ML 가상환경을 사용한다. 이 명령에서는
PYTHONPATH를 배포판 OpenCV로 덮어쓰지 않는다.

- `Q` / `Esc` / 창 닫기: 종료. Ctrl+C도 가능하다.
- `Tab`: 여러 팔레트 중 정보창에 표시할 개체 변경. 이동할 대상을 선택하는 기능이 아니다.
- `S`: 현재 화면 PNG와 추정값 JSON 저장.
- 기본 저장 폴더: `/home/sojeong/turtlebot3_ws/output/pallet_pose_preview/`.

## 화면 읽기

| 표시 | 뜻 |
|---|---|
| 사각형·0~7번 점 | 학습 모델의 팔레트 박스와 입구 모서리 예측 |
| 자홍색 십자 | 선택한 PnP 자세에서 다시 계산한 모서리 위치 |
| IPPE seed RMS | 최적화 전 초기 후보의 RMS 오차, px |
| LM refined pose shown | 현재 수치와 자홍색 십자가 LM 최적화 결과임을 표시 |
| PnP RMS | 최적화한 자세의 재투영 RMS 오차, px |
| Camera forward Z | 카메라 광학축 방향 거리, mm; 유클리드 거리와 다름 |
| Camera right X / down Y | 카메라 기준 오른쪽·아래쪽 좌표, mm |
| Body face gap (normal) | 차체 앞면에서 팔레트 면까지 수평면 법선 방향 추정 간격, mm |
| Pallet center lateral | 차체 중심선 기준 팔레트 중심 좌우 위치, 왼쪽이 + |
| Face yaw | 차체 정면과 팔레트 안쪽 법선 방향의 각도, 왼쪽이 + |
| Face tilt | 팔레트 법선이 수평면에서 벗어난 각도 |
| Alt pose | 평면 PnP의 다른 후보 거리와 오차 |
| Target body gap | 설정된 목표 170mm. 이동 명령이 아님 |

현재 차체 전면 기준은 사용자 실측 68mm다. 차체 값은 이미지 시각의 실제 TF가
조회될 때만 표시한다. TF가 없으면 카메라 좌표만 표시하며 기본 TF 값을 추측해
채워 넣지 않는다. `extrinsics_verified=false`일 때는 차체 값이 미검증 기본
장착값에 의존함을 표시한다.

**노란색 `REJECTED - diagnostic estimate only`**는 수치가 나왔더라도 정렬 기준을
통과하지 못했다는 뜻이다. 기존 1.5px 제한을 초과하는 예측도 관찰용으로 표시한다.
LM 실패 시 원래 IPPE 후보는 진단용으로만 표시하고 기하 검사를 거부한다.
각 LM 후보는 양의 깊이·거리·정면 방향·재투영 오차·방향 모호성을 검사한다.
예측점의 ROI·순서·입구 크기·신뢰도 조건도 유지한다.
녹색 `Geometry filter: PASS`는 한 프레임의 모서리/PnP 필터 통과이며, 시간에
따른 안정성·장착 검증·구동 준비 또는 이동 가능 판정이 아니다.
박스/키포인트 신뢰도는 거리 정확도를 보증하지 않는다.

이미지·CameraInfo가 오래되거나 프레임/보정 크기가 맞지 않으면 추정값을 지운다.
점 신뢰도가 부족하거나 유효한 양의 깊이 PnP 해가 없으면 자세 수치를 숨긴다.
처음 모델을 실행할 때 초기화가 느려 첫 프레임이 stale로 표시될 수 있다.

## 화면 없는 환경 및 저장 영상 시험

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/scripts/preview_pallet_pose.py \
  --headless --seconds 10
```

10초 후 마지막 화면과 JSON을 저장하고 종료한다. 실시간 모드에서는
`--seconds`를 생략하면 종료 키까지 계속 실행한다.

```bash
/home/sojeong/venv/yolov3/bin/python \
  /home/sojeong/turtlebot3_ws/scripts/preview_pallet_pose.py \
  --image /home/sojeong/turtlebot3_ws/output/pallet_pose/live_170mm_20261029/snapshot_02.jpg \
  --camera-info /home/sojeong/turtlebot3_ws/output/pallet_pose/live_170mm_20261029/snapshot_02_camera_info.json \
  --headless
```

저장 영상 모드는 ROS를 사용하지 않으며 당시 TF가 없으므로 차체 간격과 yaw는
표시하지 않는다. `--model /절대경로/다른모델.pt`, `--device cpu`,
`--output /저장폴더`로 변경할 수 있다. 모델은 pallet 1개 클래스와 동일한 8점
이름·순서를 포함한 pose 체크포인트여야 한다.

## 검증

- 저장 영상에 실제 best.pt를 적용해 박스·모서리·거부된 자세 수치를 표시했다.
- 합성 자세 복원, 거부된 점의 원시 추정 유지, 낮은 신뢰도 수치 숨김,
  차체 앞면 68mm와 카메라 위치를 포함한 간격 변환, 잘못된 보정 거부를 검사했다.
- 기존 정렬·모델 테스트를 포함해 25개 통과했다.
- 실제 ROS 카메라·CameraInfo·TF를 구독하는 10초 시험도 완료했다.
  `output/pallet_pose_preview/live_check/20261029_183533_203646.png`에
  화면을 저장했다. 해당 프레임의 재투영 오차 2.18px는 기준 미통과로 표시됐다.

## IPPE + LM 반영 (2026-10-29)

`cv2.solvePnPRefineLM`으로 각 IPPE 후보의 회전·이동을 최적화한다.
최대 100회, 종료 epsilon=1e-12이며 예측점·실물 치수·K/D는 고정한다.
실패·비정상 값·음의 깊이·오차 악화가 발생하면 통과 처리하지 않는다.
화면에는 `LIVE | IPPE + LM`, 초기 RMS와 최적화 RMS를 함께 표시한다.
이 미리보기의 PASS는 LM 결과에 대한 판정이며, 기존 제어기의 IPPE 판정과 다를 수 있다.

고정된 실제 8점 회귀 검증: RMS 2.110665px -> 약 0.79176px,
카메라 Z 269.95mm -> 약 284.88mm. 입력점이 바뀌지 않는 것도 검증했다.
최적화 실패, ROI 제한, 거리·방향·오차 제한과 모호성 거부를 포함해 관련 테스트 29개 통과.
다중 프레임 안정성이나 실제 거리 정확도, 이동 가능 여부를 보증하는 검증은 아니다.
