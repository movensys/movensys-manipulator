// [만듦 2026-10-13] WMX 엔진이 직접 남기는 로그 두 가지를 켜고 끄는 도구 (읽기만: 축을 움직이거나 설정을 바꾸지 않음)
//   1) 축 로그 (WMX LogApi, 엔진이 통신 주기마다 파일로 씀): 축 0..5 의 명령 위치·실제 위치·명령 속도·실제 속도·inPos·opState
//   2) API 로그 (WMX ApiLog): R2 노드 등이 WMX 에 보낸 API 호출(StartCSplinePos 등)과 그 입력값
// 사용
//   wmx_axis_log log <폴더> <이름> <최대 초> <멈춤 파일> [API 로그 앞부분 경로]
//       → <폴더>/<이름>.csv (축 로그), API 로그는 <앞부분>_<시각>.dat (WMX 가 시각을 붙임). <멈춤 파일>이 생기거나 최대 초가 지나면 멈춤
//   API 로그를 글자로 바꾸는 것은 WMX 도구 wmx3-apilog-parse 로 (엔진이 켜져 있어야 하고, /opt/wmx3/log/apilog/ 안의 파일만 읽음)
//   → run_demo.sh 가 앞부분을 /opt/wmx3/log/apilog/... 로 주고, 바꾼 뒤 결과 폴더로 옮긴다
//   (직접 짠 parse-api 는 시각·내용을 못 풀어서 뺐다, 2026-10-13)
// 주의
//   - 축 로그 파일은 엔진 프로세스가 쓴다. 엔진은 호스트(WMX Home)나 컨테이너(R2)에 뜰 수 있어서, 폴더는 양쪽에서 같은 경로여야 함
//     (run.sh 가 ~/workspaces/wmx_r2_logs 를 같은 경로로 연결)
//   - 엔진이 없으면 CreateDevice 가 엔진을 켜므로, 부르는 쪽(run_demo.sh)이 엔진이 떠 있을 때만 부른다
//   - CloseDevice(false): 마지막 장치여도 엔진을 끄지 않음. 객체는 지우지 않음 (소멸자가 기본값으로 닫을 수 있어서)
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <thread>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "LogApi.h"

using namespace wmx3Api;

static std::atomic<bool> g_stop{false};
static void onSig(int) { g_stop = true; }
static bool exists(const std::string &p) { struct stat st; return stat(p.c_str(), &st) == 0; }

static void err(const char *what, int e)
{
  char s[256];
  WMX3Api::ErrorToString(e, s, sizeof(s));
  fprintf(stderr, "%s 실패: err=%d (%s)\n", what, e, s);
}

int main(int argc, char **argv)
{
  if (argc < 6 || strcmp(argv[1], "log") != 0) {
    fprintf(stderr, "사용: %s log <폴더> <이름> <최대 초> <멈춤 파일> [API 로그 앞부분 경로]\n", argv[0]);
    return 2;
  }
  std::string dir = argv[2], name = argv[3], stopFile = argv[5];
  std::string apiPath = argc > 6 ? argv[6] : dir + "/" + name + "_api.dat";
  double maxSec = atof(argv[4]);
  signal(SIGINT, onSig);
  signal(SIGTERM, onSig);

  WMX3Api &wmx = *new WMX3Api();
  int e = wmx.CreateDevice("/opt/wmx3/", DeviceType::DeviceTypeNormal, 5000);
  if (e != ErrorCode::None) { err("CreateDevice", e); return 1; }
  wmx.SetDeviceName("wmx_axis_log");
  Log &log = *new Log(&wmx);

  // 비어 있는 채널 고르기 (15 부터 아래로. WMX Home 등이 낮은 번호를 쓸 수 있어서)
  int ch = -1;
  for (int c = 15; c >= 0; c--) {
    LogStatus st;
    if (log.GetLogStatus(c, &st) == ErrorCode::None && st.logState != LogState::Running) { ch = c; break; }
  }
  if (ch < 0) { fprintf(stderr, "빈 로그 채널이 없음\n"); wmx.CloseDevice(false); return 1; }

  CoreMotionLogInput in;
  in.axisSelection.axisCount = 6;
  for (int i = 0; i < 6; i++) in.axisSelection.axis[i] = i;
  in.axisOptions.commandPos = true;
  in.axisOptions.feedbackPos = true;
  in.axisOptions.commandVelocity = true;
  in.axisOptions.feedbackVelocity = true;
  in.axisOptions.inPosFlag = true;
  in.axisOptions.opState = true;
  in.axisOptions.servoOn = true;

  LogChannelOptions opt;
  opt.samplingPeriodInCycles = 1;   // 매 통신 주기 (1 ms)
  // 기록 길이. 기본값은 1000 ms 라서 첫 판은 1초만 남았다 (2026-10-13) → 최대 초만큼. 멈춤 파일이 생기면 그 전에 멈춤
  opt.samplingTimeMilliseconds = (unsigned int)(maxSec * 1000.0);
  opt.precision = 9;
  strcpy(opt.delimiter, ",");

  LogFilePathA fp;
  snprintf(fp.dirPath, sizeof(fp.dirPath), "%s", dir.c_str());
  snprintf(fp.fileName, sizeof(fp.fileName), "%s.csv", name.c_str());

  if ((e = log.SetLogFilePath(ch, &fp)) != ErrorCode::None) { err("SetLogFilePath", e); wmx.CloseDevice(false); return 1; }
  if ((e = log.SetLog(ch, &in)) != ErrorCode::None) { err("SetLog", e); wmx.CloseDevice(false); return 1; }
  if ((e = log.SetLogOption(ch, &opt)) != ErrorCode::None) { err("SetLogOption", e); wmx.CloseDevice(false); return 1; }

  bool api =log.SetApiLog(apiPath.c_str()) == ErrorCode::None && log.StartApiLog() == ErrorCode::None;
  if ((e = log.StartLog(ch)) != ErrorCode::None) { err("StartLog", e); if (api) log.StopApiLog(); wmx.CloseDevice(false); return 1; }
  printf("기록 시작: 채널 %d, 축 0..5, 1 주기마다 → %s/%s.csv, API 로그 %s\n", ch, dir.c_str(), name.c_str(),
         api ? apiPath.c_str() : "(못 켬)");
  fflush(stdout);

  auto t0 = std::chrono::steady_clock::now();
  while (!g_stop) {
    if (exists(stopFile)) break;
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (t > maxSec) { printf("최대 %.0f 초가 지나 멈춤\n", maxSec); break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  log.StopLog(ch);
  if (api) log.StopApiLog();
  // 남은 버퍼가 파일로 다 쓰일 때까지 잠깐 기다림
  LogStatus st;
  for (int i = 0; i < 60; i++) {
    if (log.GetLogStatus(ch, &st) == ErrorCode::None && st.logState != LogState::Running &&
        st.samplesWritten >= st.samplesCollected) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  printf("기록 끝: 상태 %d, 모은 %u / 쓴 %u 샘플 (%.0f ms)\n", (int)st.logState, st.samplesCollected,
         st.samplesWritten, st.millisecondsCollected);
  wmx.CloseDevice(false);
  return 0;
}
