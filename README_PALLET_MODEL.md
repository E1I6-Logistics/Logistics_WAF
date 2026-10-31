# 학습된 팔레트 모델 적용 및 관찰 시험

사용자가 지정한 `/home/sojeong/turtlebot3_ws/src/best.pt`를 사용한다.
YOLO26n Pose, `pallet` 1개 클래스, 640 입력, 8개 모서리이며 체크포인트의
`kpt_names`가 기존 PnP 모델과 일치함을 확인했다. 새 학습은 실행하지 않았다.
모델 SHA-256은 `3efb80a18d9f511c38e78b172cfb5834b2ba51fe282767386420aef58c81acba`다.

후속 치수 확인으로 중앙 받침 폭을 21.248mm, 차체 앞면→포크 끝을 약 148mm로
설정에 반영했다. 두 입구 중심 간 거리는 56.406mm이며 포크 중심 간 거리는
45mm다. CAD의 중앙 받침 `cm` 표시는 실물 축척과 다르다고 사용자가 확인했다.
사용자 요청으로 정렬 목표를 차체 앞면 기준 170mm로 적용했다. 최소 간격은
160mm, 근접 정렬 제한은 190mm이며 포크 끝의 목표 여유는 약 22mm다.
사용자 실측으로 body_front_x=0.068m를 반영했다. 카메라 장착 검증 플래그는
아직 false이며, 자세 추정과 속도 제어 경로도 실제 이동 전에 통과해야 한다.
카메라 모델 불일치는 후속 수정으로 해소했다. 로봇과 PC의 Waffle URDF에
Waffle Pi/IMX219 카메라 부분을 적용했고 실제 RGB 광학 TF [76, 0, 93]mm를
확인했다. 장착 오차의 검증은 남아 있다.
아래 영상 시험 결과는 이 치수 정밀 수정 전의 기록이다.

## 저장 영상 시험 결과 (2026-10-29)

수작업 라벨이 있는 33장 전부에서 단일 팔레트를 검출했다.
수작업 점과의 차이는 평균 3.92px, 95백분위 7.54px였다.
CPU 추론 중앙값은 최초 실행에서 약 13ms였다. 시작/영상 전송 지연은 포함하지 않는다.
학습에 쓰인 이미지인지 확인되지 않았으므로 독립 평가 성능으로 해석하면 안 된다.

예측점의 PnP 최소 재투영 오차는 2.27~4.20px로 기존 1.5px 제한을 모두 초과했다.
따라서 정렬에 사용할 수 있는 자세는 0/33장이었다. 수작업 점 자체의 PnP 통과도
16/33장이므로 모델 오차뿐 아니라 앞면 모서리 라벨 정의, 실측 치수, 보정을 함께
확인해야 한다. 이 결과만으로 어느 항목이 원인인지 확정하지 않는다.
실측 거리·각도의 정답은 없어 mm 단위 정확도는 평가하지 않았다.

도메인 11에서 약 15초 동안 실제 JPEG/CameraInfo를 구독하는 관찰 시험도
실행했다. 영상 처리까지 진행했으나 유효한 자세를 확보하지 못해 시작 제한
시간으로 종료했다. Nav2 목표나 속도 명령은 발행하지 않았다.
[실시간 관찰 로그](output/pallet_pose/20261029_best_manual33/live_observe_network.log)
에 실행 결과를 보관했다. 오프라인 관련 테스트는 19개 통과했다.

- [이미지별 진단 결과](output/pallet_pose/20261029_best_src_manual33/report.json)
- [예측점 비교 예시](output/pallet_pose/20261029_best_src_manual33/overlays/pallet_training_0027.jpg)
- 녹색 점: 모델 예측, 자홍색 원: 수작업 라벨, 숫자: 모서리 인덱스.

## 실시간 관찰 (이동 없음)

기존 `/home/sojeong/venv/yolov3` 환경의 PyTorch 2.14.0+cu130,
Ultralytics 8.4.138을 사용한다. 이 환경의 OpenCV/NumPy와 ROS import를 확인했다.
현재 NVIDIA 드라이버가 동작하지 않아 `pose_device: cpu`로 설정했다.
시스템 패키지·드라이버는 변경하지 않았다.

```bash
cd /home/sojeong/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=11 /home/sojeong/venv/yolov3/bin/python \
  src/pallet_alignment.py --config config/pallet_alignment_yolo.yaml --show
```

그래픽 화면 없는 터미널에서는 `--show`를 생략한다. 이 명령은 카메라가 이미
JPEG와 CameraInfo를 발행하고 있어야 한다. 관찰 모드는 속도 publisher를 만들지 않는다.
노란 점은 모델 예측이며 자세를 승인했다는 뜻이 아니다. 녹색 윤곽과 거리 표시는
PnP까지 통과한 경우에만 나타난다. 검출 실패·모호한 다중 팔레트·신뢰도 부족·오래된
프레임·PnP 불일치에서는 기존 정지 조건을 유지한다.
기본 contour 설정은 그대로이며 모델용 설정을 명시해야 모델을 사용한다.

## 실제 정렬을 진행하기 위한 미해결 항목

1. 실제 팔레트 앞면 8점과 모델 예측점을 비교하고, 치수/CameraInfo가 해당
   팔레트와 카메라 설정에 맞는지 확인한다. 임계값만 늘려 오차를 숨기지 않는다.
2. 서로 다른 실제 거리·좌우 위치·각도에서 오차를 측정한다. 검출 점수나
   PnP 재투영 오차만으로 차체 간격 오차를 대신 평가하지 않는다.
3. 카메라→base_link 장착 TF 및 base_link→차체 앞면 거리 `body_front_x`를 실측한다.
   body_front_x는 사용자 실측 0.068m로 입력했다.
   `extrinsics_verified: false` 상태에서는 이동을 차단한다.
4. 위 항목을 검증한 뒤 기존 README_PALLET_ALIGNMENT.md의 속도 경로와
   `--execute` 시험 절차를 따른다. 이 작업에서는 실물 이동을 실행하지 않았다.

## 추가 학습이 필요할 경우

수집 원본 및 수작업 JSON은 변경하지 않았다. 변환 도구는 모든 점의 Group ID와
라벨 이름, 좌표, 이미지 크기, 원본 해시를 검사한다. 누락된 점은 visibility=0,
`flags.occluded=true` 점은 visibility=1, 보이는 점은 visibility=2다.
빈 JSON은 `flags.background=true`가 있어야 배경으로 취급한다.
검사는 위치 정답 여부를 보증하지 않는다.

```bash
# 새 출력 폴더를 사용한다. 현재 33장에서는 18 train / 5 val / 10 test가 된다.
/usr/bin/python3 scripts/prepare_pallet_pose.py \
  --source datasets/pallet/curated/manual_01/dataset --raw datasets/pallet/raw \
  --output datasets/pallet/processed/manual33_session_split \
  --val-sessions 20261028_training_03 --test-sessions 20261028_training_04

/home/sojeong/venv/yolov3/bin/python scripts/train_pallet_pose.py \
  --data datasets/pallet/processed/manual33_session_split/data.yaml \
  --model yolo11n-pose.pt --device cpu --epochs 100 \
  --output runs/pallet_pose/manual33_trial
```

위 명령은 재학습용 예시이며 실행하지 않았다. 18장 학습은 파이프라인 시험용이다.
실제 일반화 성능에는 다른 배치·조명·거리·배경과 팔레트 없는 장면이 추가로 필요하다.
현재 모델과 겹치지 않는 새 촬영을 별도 평가 세트로 보관한다.
데이터 형식은 [Ultralytics Pose 데이터셋 문서](https://docs.ultralytics.com/datasets/pose/)를 따른다.

```bash
# 같은 모델/보정/라벨로 오프라인 진단 재실행 (출력은 새 폴더)
/home/sojeong/venv/yolov3/bin/python scripts/evaluate_pallet_pose.py \
  --config config/pallet_alignment_yolo.yaml \
  --manifest datasets/pallet/processed/20261029_manual33_audit/manifest.json \
  --output output/pallet_pose/new_diagnostic

source /opt/ros/jazzy/setup.bash
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH \
  /usr/bin/python3 -m unittest discover -s tests -p 'test_pallet*.py' -q
```
