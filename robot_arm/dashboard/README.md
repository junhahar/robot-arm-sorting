# Sambo Robot Arm Dashboard

삼보모터스 로봇팔 자동 분류 시스템의 발표 및 작업자용 대시보드입니다.

## 실행 방법

1. Python 3를 설치합니다.
2. OpenAI API를 사용할 경우 PowerShell에서 개인 API 키를 등록합니다.

```powershell
setx OPENAI_API_KEY "본인이_발급받은_API_키"
```

3. 새 PowerShell 창을 열거나 PC를 재부팅합니다.
4. `대시보드_실행.bat`를 더블클릭합니다.
5. 브라우저에서 `http://127.0.0.1:8766/`에 접속합니다.

API 키는 저장소에 포함하지 않습니다. 팀원별로 개인 환경변수에 등록해야 합니다.

## 포함 기능

- 티칭 모드, 자동 운전, 수동 시험 화면
- 실제 STL 기반 3D 로봇 모델과 J1~J6 실시간 각도 연동
- J2·J3 연동 수동 제어와 HOME 복귀
- MPU 팔 동작 추정, 그리퍼 카메라, YOLO 데모 화면
- CAN, 모터, 그리퍼, ToF, 이벤트 로그 상태 표시
- OpenAI API 기반 작업자용 AI 설명 도우미

실제 장비 WebSocket 연결이 없으면 화면 상단에 `DEMO MODE`가 표시됩니다.
