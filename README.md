# 개요
2025년 7~8월간 진행된 KIRO의 방수총 제어 프로젝트 코드 및 실험 데이터입니다.


# 폴더 설명

- CnSim_Linux_original: 처음 방수총 받았을 때 원본 \
- CnSim_Linux: (8월 30일 01:19) 현재 방수총 컴퓨터에 올라와있는 최신 버전.

- csv_data: 각 trial의 csv data를 정리해놓은 폴더입니다. 각 trial의 parameter는 다음의 링크에서 확인할 수 있는 Notion 페이지에 있습니다.
- https://www.notion.so/cdsl-uos-wiki/2536252d2a7d8047b271ee32672c7fcf?source=copy_link


# 코드 설명
- reference generator:
  - 시작점과 끝점 위치를 가지고 end effector의 position, velocity, and accleration의 시간에 따른 reference를 생성하는 모듈입니다.
  - 5차 polynomial로 reference를 생성하는 방법을 이용하였습니다.

- model dynamics:
  - classicial D-H parameter를 이용하여 2 link manipulator의 kinematics를 구하였습니다.
  - 이 kinematics를 바탕으로 Euler-Lagrange equation을 통해 dynamics를 구하였습니다.

- controller:
  - 통신 및 기어비 등은 존재했지만 control input은 비어있던 코드입니다. (Cnsim_linux_original을 확인하면 알 수 있음)
  - PD controller, FL controller, DOB 가 본 코드 내에 각각의 함수로 구현되어 있습니다.
  - 내부에서 controller mode 변경을 통해서 PD / FL / FL + DOB 제어로 변경할 수 있습니다.
