# [Manual] Visual Studio 기반 서버 접속 가이드

이 메뉴얼은 Visual Studio 환경에서 주어진 서버에 접속하기 위해 따라야하는 과정을 설명합니다.

## 1. 전제 조건 (Prerequisites)
* 호스트 OS: Ubuntu 22.04 LTS
* Visual Studio 설치가 되어있어야 함.

## 2. Visual Studio 환경 설정 (Environment Setup)

### 2.1 Remote SSH 설치

Visual Studio의 EXtension 탭에서 Remote SSH를 설치.

[추후 사진 첨부]
![how to 1.png](capture/how to server/how to 1.png)

### 2.2 접속 메뉴 열기
VS Code 왼쪽 아래 구석에 '><' 모양의 아이콘을 클릭하거나 키보드에서 F1 키를 누르고 Remote-SSH : Connect to Host를 검색.

[추후 사진 첨부2]

### 2.3 SSH Host 설정 변경
이후 편리한 연결을 위하여 config 파일에 서버 정보 등록. Connect to Host 선택 잏 COnfigure SSH Hosts 선택.
config 파일 경로는 /home/(유저 이름)/.ssh/config


```
Host sysai-server2
	HostName 210.110.250.33
	User dg
	Port 7859
```

[추후 사진 첨부3]

### 2.4 서버 연결
서버 선택 후 클릭하면 새로운 Visual Studio 창이 뜨면서 비밀번호 입력 요구.
비밀번호를 입력하면 서버 접속 성공. (비밀번호는 피지컬AI 카톡방 공지 참고)

[추후 사진 첨부4]
[추후 사진 첨부5]

### 2.5 서버 연결 종료
VS Code의 왼쪽 아래의 >< 아이콘을 누르고, Close Remote Connection 클릭.

[추후 사진 첨부6]
