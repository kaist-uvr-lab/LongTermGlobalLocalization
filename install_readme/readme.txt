* 전체 다 MT >> MD로 빌드. 

1. Pangolin
1) github에서 .zip파일이 아닌 git clone을 설치. 
2) cmake할때 openni2, uvc 사용 해제 할것 
3) cmake할때 커맨드 라인에서 cmake -G "Visual Studio 14 2015 Win64" ..
   그 후, build 폴더에서 cmake --build --config Release (glew 등 자동 설치 위해)
4) VS에서 MT가 아닌 MD로 빌드할것. 

2. DBoW2, g2o 빌드. 
1) 짜잘한 에러들은 구글링하면 바로 나옴. 
2) .dll이 아닌 .lib으로 빌드할것.

3. Boost 빌드 >> 마찬가지로 MD로 빌드할것.

4. ORB-SLAM2 빌드
1) Linker 설정. (boost lib 위치 추가)
2) .lib으로 빌드할것.

5. LSM 빌드
 
6. LTGL 빌드. 




