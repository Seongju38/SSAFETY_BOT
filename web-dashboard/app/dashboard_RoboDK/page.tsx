"use client";

import { useState, useRef, useEffect } from "react";

const EVENT_OPTIONS = [
  { value: "exhaustion", label: "탈진" },
  { value: "collision", label: "충돌/골절" },
  { value: "bleeding", label: "출혈" },
  { value: "chronic", label: "기저질환 의심" },
] as const;

type KitConfig = {
  title: string;
  items: string[];
};

const KIT_PRESETS: Record<string, KitConfig> = {
  exhaustion: {
    title: "추천 키트: 탈진",
    items: ["수분 공급 키트", "전해질 음료", "냉/온찜질팩"],
  },
  collision: {
    title: "추천 키트: 충돌/골절",
    items: ["부목 세트", "탄력 붕대", "얼음팩"],
  },
  bleeding: {
    title: "추천 키트: 출혈",
    items: ["지혈 패드", "압박 붕대", "소독제"],
  },
  chronic: {
    title: "추천 키트: 기저질환 의심",
    items: ["혈압계", "혈당 측정 키트", "기본 구급 키트"],
  },
};

const EVENT_TO_DISEASE: Record<string, "EXH" | "CHR" | "BLE" | "COL"> = {
  exhaustion: "EXH",
  collision: "COL",
  bleeding: "BLE",
  chronic: "CHR",
};

type ServerStatus = {
  fallen: boolean;
  fall_reason: string;
  fall_event_id: number;
  fall_started_ms: number | null;
  busy: boolean;
  last_pose: {
    x: number;
    y: number;
    z: number;
    roll: number;
    pitch: number;
    yaw: number;
  } | null;
  last_dispatch: any;

  // ✅ stage 추가
  dispatch_stage:
    | "IDLE"
    | "PICKING"
    | "LOADING"
    | "NAVIGATING"
    | "ARRIVED"
    | "DONE"
    | "ERROR";
  dispatch_stage_id: number;
  dispatch_disease: string | null;
};

const SERVER_BASE = "http://localhost:8000";

export default function DashboardPage() {
  const [selectedEvent, setSelectedEvent] = useState<string>("exhaustion");
  const [logs, setLogs] = useState<string[]>([]);
  const [isSending, setIsSending] = useState(false);

  const [serverStatus, setServerStatus] = useState<ServerStatus | null>(null);

  const [soundEnabled, setSoundEnabled] = useState(false);
  const lastPlayedRef = useRef<number | null>(null);
  const currentEventRef = useRef<number | null>(null);

  // ✅ stage 음성 중복 방지용
  const lastStageIdRef = useRef<number | null>(null);

  const currentKit = KIT_PRESETS[selectedEvent];

  const stopVoice = () => {
    window.speechSynthesis.cancel();

    if (currentEventRef.current != null) {
      lastPlayedRef.current = currentEventRef.current;
      currentEventRef.current = null;
    }
  };

  const speak = (msg: string) => {
    if (!soundEnabled) return;
    window.speechSynthesis.cancel();

    const voices = window.speechSynthesis.getVoices();
    const googleVoice = voices.find((v) => v.name === "Google 한국의");

    const u = new SpeechSynthesisUtterance(msg);
    u.lang = "ko-KR";

    if (googleVoice) {
      u.voice = googleVoice;
    }

    window.speechSynthesis.speak(u);
  };

  useEffect(() => {
    const timer = setInterval(async () => {
      try {
        const res = await fetch(`${SERVER_BASE}/status`, { cache: "no-store" });
        if (!res.ok) return;

        const data: ServerStatus = await res.json();
        setServerStatus(data);

        // ====== 1) FALL 이벤트 음성 ======
        const eventId = data.fall_event_id;
        if (eventId) {
          if (
            lastPlayedRef.current !== eventId &&
            currentEventRef.current !== eventId
          ) {
            currentEventRef.current = eventId;

            if (!soundEnabled) {
              const now = new Date().toLocaleTimeString();
              setLogs((prev) => [`[${now}] 🚨 쓰러짐 감지(음성 OFF)`, ...prev]);
              lastPlayedRef.current = eventId;
              currentEventRef.current = null;
            } else {
              const now = new Date().toLocaleTimeString();
              setLogs((prev) => [
                `[${now}] 🚨 쓰러짐 감지 → 음성 안내 재생`,
                ...prev,
              ]);

              window.speechSynthesis.cancel();
              speak(
                "작업장에 쓰러진 사람이 감지되었습니다. 필요한 응급 키트를 선택해 주세요."
              );
              lastPlayedRef.current = eventId;
              currentEventRef.current = null;
            }
          }
        }

        // ====== 2) DISPATCH 단계 음성 ======
        const stageId = data.dispatch_stage_id;
        const stage = data.dispatch_stage;

        if (
          stageId != null &&
          stageId !== 0 &&
          lastStageIdRef.current !== stageId
        ) {
          lastStageIdRef.current = stageId;

          let msg = "";
          if (stage === "PICKING") msg = "응급 키트를 집는 중입니다.";
          else if (stage === "LOADING")
            msg = "키트를 터틀봇에 적재하고 있습니다.";
          else if (stage === "NAVIGATING")
            msg = "터틀봇이 환자 위치로 이동 중입니다.";
          else if (stage === "ARRIVED")
            msg = "터틀봇이 환자 근처에 도착했습니다.";
          else if (stage === "DONE")
            msg = "응급 키트 전달 과정이 완료되었습니다.";
          else if (stage === "ERROR")
            msg = "오류가 발생했습니다. 로그를 확인해 주세요.";

          if (msg) {
            const now = new Date().toLocaleTimeString();
            setLogs((prev) => [`[${now}] 🔈 ${msg}`, ...prev]);
            speak(msg);
          }
        }
      } catch {
        // 서버 죽었을 때 조용히 무시
      }
    }, 500);

    return () => clearInterval(timer);
  }, [soundEnabled]);

  const handleDispatch = async () => {
    try {
      setIsSending(true);

      const disease_key = EVENT_TO_DISEASE[selectedEvent] ?? "CPR";

      const res = await fetch(`${SERVER_BASE}/dispatch`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ disease_key, fallen: true }),
      });

      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(err.detail ?? `HTTP ${res.status}`);
      }

      const now = new Date().toLocaleTimeString();
      const label =
        EVENT_OPTIONS.find((e) => e.value === selectedEvent)?.label ??
        selectedEvent;

      setLogs((prev) => [
        `[${now}] ✅ 로봇 파견 시작 요청 - 유형: ${label} (disease_key=${disease_key})`,
        ...prev,
      ]);
    } catch (err) {
      const now = new Date().toLocaleTimeString();
      setLogs((prev) => [
        `[${now}] ❌ 로봇 파견 실패: ${(err as Error).message}`,
        ...prev,
      ]);
    } finally {
      setIsSending(false);
    }
  };

  const fallen = !!serverStatus?.fallen;
  const busy = !!serverStatus?.busy;

  return (
    <main className="min-h-screen bg-slate-950 text-slate-50 flex flex-col">
      <header className="border-b border-slate-800 px-6 py-4 flex items-center">
        <h1 className="text-xl font-semibold">SSAFETY BOT · 관리자 대시보드</h1>

        <div className="ml-auto flex items-center gap-2">
          <button
            onClick={() => setSoundEnabled(true)}
            className="px-3 py-2 rounded-lg bg-slate-800 text-slate-100 text-sm hover:bg-slate-700 transition-colors"
          >
            🔊 알림 소리 켜기
          </button>

          <button
            onClick={() => setSoundEnabled(false)}
            className="px-3 py-2 rounded-lg bg-slate-800 text-slate-100 text-sm hover:bg-slate-700 transition-colors"
          >
            🔇 알림 소리 끄기
          </button>

          <button
            onClick={stopVoice}
            className="px-3 py-2 rounded-lg bg-red-600 text-white text-sm hover:bg-red-700 transition-colors"
          >
            ⏹ 음성 정지
          </button>
        </div>
      </header>

      <div className="flex-1 grid grid-cols-1 lg:grid-cols-3 gap-4 p-6">
        <section className="lg:col-span-2 bg-slate-900 rounded-2xl border border-slate-800 p-4 flex flex-col">
          <div className="flex items-center justify-between mb-3">
            <h2 className="text-sm font-medium text-slate-200">
              낙상 감지 실시간 영상
            </h2>

            <div className="text-xs text-slate-300 flex items-center gap-2">
              <span
                className={`px-2 py-1 rounded-md border ${
                  fallen
                    ? "bg-red-600/20 border-red-500 text-red-200"
                    : "bg-emerald-600/10 border-emerald-500 text-emerald-200"
                }`}
              >
                {fallen
                  ? `🚨 FALL (${serverStatus?.fall_reason ?? "?"})`
                  : "✅ 정상"}
              </span>

              <span
                className={`px-2 py-1 rounded-md border ${
                  busy
                    ? "bg-amber-600/10 border-amber-500 text-amber-200"
                    : "bg-slate-800 border-slate-700 text-slate-200"
                }`}
              >
                {busy ? "로봇 동작 중" : "대기"}
              </span>

              <span className="px-2 py-1 rounded-md border bg-slate-800 border-slate-700 text-slate-200">
                stage: {serverStatus?.dispatch_stage ?? "IDLE"}
              </span>
            </div>
          </div>

          <div className="flex-1 rounded-xl border border-dashed border-slate-700 flex items-center justify-center text-slate-500 text-sm overflow-hidden">
            <img
              src={`${SERVER_BASE}/stream`}
              alt="RoboDK stream"
              className="w-full h-full object-contain rounded-xl"
            />
          </div>

          <div className="mt-3 text-[11px] text-slate-400">
            {serverStatus?.last_pose ? (
              <div className="flex flex-wrap gap-x-4 gap-y-1">
                <span>x: {serverStatus.last_pose.x.toFixed(1)}</span>
                <span>y: {serverStatus.last_pose.y.toFixed(1)}</span>
                <span>z: {serverStatus.last_pose.z.toFixed(1)}</span>
                <span>roll: {serverStatus.last_pose.roll.toFixed(1)}°</span>
                <span>pitch: {serverStatus.last_pose.pitch.toFixed(1)}°</span>
                <span>yaw: {serverStatus.last_pose.yaw.toFixed(1)}°</span>
              </div>
            ) : (
              <span>pose: (no data)</span>
            )}
          </div>
        </section>

        <section className="bg-slate-900 rounded-2xl border border-slate-800 p-4 flex flex-col gap-4">
          <div>
            <h2 className="text-sm font-medium text-slate-200 mb-2">
              1. 상황 선택
            </h2>
            <select
              className="w-full rounded-lg bg-slate-950 border border-slate-700 px-3 py-2 text-sm outline-none focus:ring-2 focus:ring-emerald-500"
              value={selectedEvent}
              onChange={(e) => setSelectedEvent(e.target.value)}
            >
              {EVENT_OPTIONS.map((event) => (
                <option key={event.value} value={event.value}>
                  {event.label}
                </option>
              ))}
            </select>

            <p className="mt-2 text-[11px] text-slate-500">
              서버 disease_key 매핑:{" "}
              <span className="text-slate-300 font-medium">
                {EVENT_TO_DISEASE[selectedEvent] ?? "CPR"}
              </span>
            </p>
          </div>

          <div>
            <h2 className="text-sm font-medium text-slate-200 mb-2">
              2. 추천 응급 키트
            </h2>
            <div className="rounded-lg bg-slate-950 border border-slate-700 px-3 py-3 text-sm">
              <div className="font-semibold mb-1">{currentKit.title}</div>
              <ul className="list-disc list-inside text-slate-300 text-xs space-y-0.5">
                {currentKit.items.map((item) => (
                  <li key={item}>{item}</li>
                ))}
              </ul>
            </div>
          </div>

          <div className="mt-auto">
            <button
              onClick={handleDispatch}
              disabled={isSending || busy || !fallen}
              className="w-full rounded-lg bg-emerald-500 hover:bg-emerald-400 disabled:bg-emerald-700 disabled:cursor-not-allowed text-slate-950 font-semibold py-2 text-sm transition"
              title={
                !fallen
                  ? "쓰러짐 감지 후 파견 가능"
                  : busy
                  ? "로봇 동작 중"
                  : ""
              }
            >
              {isSending
                ? "로봇 파견 중…"
                : busy
                ? "로봇 동작 중…"
                : "로봇 파견"}
            </button>

            <p className="mt-2 text-[11px] text-slate-500">
              * <span className="text-slate-300">쓰러짐(FALL)</span>이 감지된
              상태에서만 파견 버튼이 활성화됩니다.
            </p>
          </div>
        </section>
      </div>

      <section className="border-t border-slate-800 px-6 py-3 bg-slate-950">
        <h2 className="text-xs font-medium text-slate-300 mb-1">이벤트 로그</h2>
        {logs.length === 0 ? (
          <p className="text-[11px] text-slate-500">
            아직 이벤트 로그가 없습니다.
          </p>
        ) : (
          <ul className="max-h-28 overflow-auto text-[11px] text-slate-300 space-y-0.5">
            {logs.map((log, idx) => (
              <li key={idx}>{log}</li>
            ))}
          </ul>
        )}
      </section>
    </main>
  );
}
