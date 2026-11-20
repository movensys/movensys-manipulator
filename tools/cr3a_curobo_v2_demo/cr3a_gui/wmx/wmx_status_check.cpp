// [만듦 2026-10-13] 컨테이너 → 호스트 WMX 엔진 연결 확인 (읽기만 함). run.sh check 4번, sim/axis_demo.sh 가 컴파일해서 실행한다.
// 하는 일 : CreateDevice(엔진에 붙기) → 엔진 상태 / 붙어 있는 장치 / 불러온 플랫폼 모듈 읽기 → CloseDevice(false)(떼기만)
// 안 하는 일: 통신 시작/정지, 서보, 알람 해제, 파라미터 쓰기, 엔진 정지
// 주의 : CreateDevice 는 엔진이 꺼져 있으면 엔진을 켠다 → 부르는 쪽(run.sh, axis_demo.sh)은 엔진이 이미 살아 있을 때만 부른다.
//        CloseDevice 의 기본값(exitNoDev=true)은 "마지막 장치면 엔진도 끔" 이라서 false 로 부른다.
// [추가 2026-10-13] --require-simu : 엔진이 불러온 플랫폼이 시뮬레이터(simu_platform.so) 하나뿐이 아니면 종료 코드 3.
//        시뮬레이션 예제가 실제 로봇(EtherCAT 플랫폼)에 명령을 보내지 않게 막는 두 번째 안전장치 (첫 번째는 Module.ini 검사).
#include <cstdio>
#include <cstring>

#include "WMX3Api.h"

using namespace wmx3Api;

static const char *stateName(EngineState::T s)
{
  switch (s) {
    case EngineState::Idle:          return "Idle";
    case EngineState::Running:       return "Running";
    case EngineState::Communicating: return "Communicating";
    case EngineState::Shutdown:      return "Shutdown";
    default:                         return "Unknown";
  }
}

int main(int argc, char **argv)
{
  bool requireSimu = (argc > 1 && strcmp(argv[1], "--require-simu") == 0);

  int major = 0, minor = 0, rev = 0, fix = 0;
  WMX3Api::GetLibVersion(&major, &minor, &rev, &fix);
  printf("WMX3 API 라이브러리 %d.%d.%d.%d\n", major, minor, rev, fix);

  // [고침 2026-10-13, 리뷰 지적] 소멸자가 CloseDevice() 를 기본값(exitNoDev=true = 마지막 장치면 엔진도 끔)으로 부를 수 있어서,
  //  객체를 new 로 만들고 지우지 않는다 (프로세스가 끝나면 OS 가 정리. 떼기는 아래 CloseDevice(false) 로 직접 함)
  WMX3Api &wmx = *new WMX3Api();
  char msg[256];
  int err = wmx.CreateDevice("/opt/wmx3/", DeviceType::DeviceTypeNormal, 5000);
  if (err != ErrorCode::None) {
    WMX3Api::ErrorToString(err, msg, sizeof(msg));
    printf("엔진에 붙기 실패: CreateDevice err=%d (%s)\n", err, msg);
    return 1;
  }
  wmx.SetDeviceName("wmx_r2_status_check");
  printf("엔진에 붙음 (CreateDevice OK)\n");

  EngineStatus st;
  err = wmx.GetEngineStatus(&st);
  if (err == ErrorCode::None) {
    printf("엔진 상태: %s (error=%u, 주기 %d us)\n", stateName(st.state), st.error,
      st.numOfInterrupts > 0 ? st.interrupts[0].cycleTimeMicroseconds : -1);
  } else {
    WMX3Api::ErrorToString(err, msg, sizeof(msg));
    printf("GetEngineStatus err=%d (%s)\n", err, msg);
  }

  DevicesInfoA devs;
  if (wmx.GetAllDevices(&devs) == ErrorCode::None) {
    printf("붙어 있는 장치 %u개:", devs.count);
    for (unsigned int i = 0; i < devs.count; i++) {
      printf(" [%u]%s", devs.devices[i].id, devs.devices[i].name);
    }
    printf("\n");
  }

  // 엔진이 실제로 불러온 플랫폼 (Module.ini 를 나중에 바꿨어도, 떠 있는 엔진은 켤 때의 설정을 쓴다)
  int numPlatform = 0, numNonSimu = 0;
  static ModulesInfoA mods;  // 64개 모듈 정보라 스택 대신 정적 영역
  err = wmx.GetModulesInfo(&mods);
  if (err == ErrorCode::None) {
    printf("플랫폼 모듈:");
    for (int i = 0; i < mods.numOfModule; i++) {
      if (mods.modules[i].type != ModuleType::Platform) continue;
      numPlatform++;
      bool simu = strstr(mods.modules[i].fileName, "simu") != nullptr;
      if (!simu) numNonSimu++;
      printf(" [%s %s%s]", mods.modules[i].moduleName, mods.modules[i].fileName, simu ? "" : " ← 시뮬레이터 아님");
    }
    printf("%s\n", numPlatform == 0 ? " 없음" : "");
  } else {
    WMX3Api::ErrorToString(err, msg, sizeof(msg));
    printf("GetModulesInfo err=%d (%s)\n", err, msg);
  }

  err = wmx.CloseDevice(false);
  printf("떼기 (CloseDevice(false)) %s\n", err == ErrorCode::None ? "OK" : "실패");

  if (requireSimu) {
    if (numPlatform >= 1 && numNonSimu == 0) {
      printf("시뮬레이터 확인: OK (플랫폼은 simu 뿐)\n");
      return 0;
    }
    printf("시뮬레이터 확인: 실패 (플랫폼 %d개 중 시뮬레이터 아닌 것 %d개) → 중단\n", numPlatform, numNonSimu);
    return 3;
  }
  return 0;
}
