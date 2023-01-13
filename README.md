# 토렌즈(토일렛 프렌즈)
## 서비스 소개
> Toilet + Friend
- 유치원, 어린이집대상으로 양치, 세안, 손씻기를 교육 및 교정하는 서비스
- 스마트 미러로 만들어 아동들의 흥미를 유발하고 아동들이 좋아하는 캐릭터로 관심을 끌게하는 서비스
- [UCC 바로가기](추후 작성)

## 팀원 소개 및 담당 역할
- 김상래 (팀장) : 임베디드 개발, 프로젝트 관리
- 이영도 (팀원) : 임베디드 개발
- 이준호 (팀원) : 임베디드 개발
- 여도현 (팀원) : Front-End 개발,
- 최선호 (팀원) : Back-End 개발, 

## 기술 스택
- IoT : OpenCV, RaspberryPI
- Frontend : React, Redux
- Backend : Spring, MySQL
- Infra : Jenkins, Amazon S3, Nginx

<br>

## 서비스

### 1. 대기화면에서 

사진 추가

### 2. 손의 모션을 캡쳐해서 양치, 세안, 손씻기 버튼 클릭

사진 추가

### 3. 각 교육 진행

사진 추가

### 4. AR기능

사진 추가

### 5. 교육 종료 후 LED 전구를 활용하여 효과 연출

사진 추가



<br>

## DETAIL

### 0. Conventions

- **git commit message**

  > type: Subject 형태로 작성하기

  type

  - master : 제품으로 출시될 수 있는 브렌치. 최선의 상태 유지
  - Feat : 새로운 기능 추가
  - Fix : 버그 수정
  - Docs : 문서 수정
  - Refactor : 코드 리팩토링
  - Design : CSS 등 사용자 UI 디자인 변경
  - Comment : 필요한 주석 추가 및 변경
  - Rename : 파일 또는 폴더 명을 수정
  - Remove : 파일을 삭제하는 작업만 수행
  - HOTFIX : 급하게 치명적인 버그를 수정

  Subject

  - 50자를 넘기지 말 것
  - 대문자로 시작하며 마침표는 적지 않을 것
  - 명령조로 말할것. 예를들면 changed, changes 가 아닌 change

- **GitLab branch 전략**

  - master : 제품으로 출시될 수 있는 브렌치. 최선의 상태 유지
  - Feat : 새로운 기능 추가
  - Fix : 버그 수정
  - Docs : 문서 수정
  - Refactor : 코드 리팩토링
  - Design : CSS 등 사용자 UI 디자인 변경
  - Comment : 필요한 주석 추가 및 변경
  - Rename : 파일 또는 폴더 명을 수정
  - Remove : 파일을 삭제하는 작업만 수행
  - HOTFIX : 급하게 치명적인 버그를 수정

- **Jira**

  이슈생성

  - 월요일에 주 단위 스프린트 이슈들을 생성
  - 일주일에 40시간 이상 이슈 생성 및 처리

  이슈 완료

  - 하나의 티켓은 되도록 하나의 커밋으로 처리

- **Style Guide**

  FE

  - var대신 const, let 사용하기
  - 가능한 Arrow Function으로 함수식 작성
  - 변수명 : file_scope 상수는 UPPER_SNAKE_CASE, 그 외엔 camelCase. Boolean변수는 앞에 is 붙이기.
  - 클래스(생성자)명 : 파스칼 케이스를 이용한다. PascalCase
  - 클래스(HTML)명 : 케밥 케이스를 이용한다. kebal-case

  BE

  - 변수명, 인스턴스명: camelCase
  - 상수 변수명(static final): 대문자, 띄어쓰기는 _ 사용
  - 클래스명, 생성자명: PascalCase
  - 함수명: camelCase(동사 + 명사로 구성)
  - Mysql table, column name: snake_case
  - photogroup 은 한단어로 표기하기

<br>

### 1. Planning & Design

- 아이디어 기능 정리



- 와이어 프레임


- ERD



- API


- 아키텍처



<br>

### 2. Development & Test

- FE Directory: pages에서 component 들을 활용하여 페이지 구성, store에서 redux 관리, api에서 request functions 관리

```
├─public
│      favicon.ico
│      index.html
│      manifest.json
│      robots.txt
│
└─src
    │  App.css
    │  App.js
    │  App.test.js
    │  index.css
    │  index.js
    │  reportWebVitals.js
    │  setupTests.js
    │
    ├─api
    │      api.js
    │      community.js
    │      mypage.js
    │      s3.js
    │      user.js
    │
    ├─assets
    │  │  defaultProfile.png
    │  │  img404.png
    │  │  logo.png
    │  │
    │  └─UI
    │          add.png
    │          back.png
    │          ...
    │
    ├─components
    │  ├─Community
    │  │      Comments.css
    │  │      Comments.js
    │  │      CommentsEdit.js
    │  │      CommentsList.js
    │  │      CommentsNew.js
    │  │      CommunityCarousel.css
    │  │      CommunityCarousel.js
    │  │      CommunityListFrame.css
    │  │      CommunityListFrame.js
    │  │      CommunityListPhoto.css
    │  │      CommunityListPhoto.js
    │  │      SharePost.css
    │  │      SharePost.js
    │  │
    │  ├─Layout
    │  │      Footer.css
    │  │      Footer.js
    │  │      Header.css
    │  │      Header.js
    │  │      Layout.css
    │  │      Layout.js
    │  │
    │  ├─MyPage
    │  │      ActivityTabs.css
    │  │      ActivityTabs.js
    │  │      FollowerList.js
    │  │      FollowingList.js
    │  │      MyGallery.js
    │  │      MypageProfile.css
    │  │      MypageProfile.js
    │  │      Search.js
    │  │
    │  ├─ScrollCalendar
    │  │      Calendar.css
    │  │      Calender.js
    │  │      ScrollCalendar.css
    │  │      ScrollCalendar.js
    │  │
    │  ├─User
    │  │      Profile.css
    │  │      Profile.js
    │  │
    │  └─Utils
    │          FloatingBtn.js
    │          ImageEdit.js
    │          ModalBasic.js
    │          ModalConfirm.js
    │          MsgEdit.js
    │          Pagination.js
    │
    ├─pages
    │      Ar.js
    │      Community.css
    │      Community.js
    │      Download.css
    │      Download.js
    │      FrameEdit.css
    │      FrameEdit.js
    │      Kakao.css
    │      Kakao.js
    │      Login.css
    │      Login.js
    │      Main.css
    │      Main.js
    │      MyPage.css
    │      Mypage.js
    │      NotFound404.css
    │      NotFound404.js
    │      ProfileDelete.js
    │      ProfileEdit.css
    │      ProfileEdit.js
    │      ProfileEditPw.js
    │      Signup.js
    │
    └─store
        │  store.js
        │
        └─modules
                community.js
                mypage.js
                user.js
```

- Jenkins: gitlab dev branch에 merge(or push)시 자동 재빌드

- Jira: 주단위로 월요일날 회의를 통해 이슈를 만들고 그 주 금요일 혹은 일요일까지 대부분 완료


<br>

### 3. Opperation

- 아동들을 찾아가서 우리 시제품을 사용하게끔 하는 사진 등을 추가
- 

<br>

### 4. ETC

[깃랩](깃랩 추후 추가)
