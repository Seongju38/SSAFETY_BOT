'use client';

import { useState } from 'react';

const EVENT_OPTIONS = [
  { value: 'exhaustion', label: '탈진' },
  { value: 'collision', label: '충돌/골절' },
  { value: 'bleeding', label: '출혈' },
  { value: 'chronic', label: '기저질환 의심' },
];

type KitConfig = {
  title: string;
  items: string[];
};

const KIT_PRESETS: Record<string, KitConfig> = {
  exhaustion: {
    title: '추천 키트: 탈진',
    items: ['수분 공급 키트', '전해질 음료', '냉/온찜질팩'],
  },
  collision: {
    title: '추천 키트: 충돌/골절',
    items: ['부목 세트', '탄력 붕대', '얼음팩'],
  },
  bleeding: {
    title: '추천 키트: 출혈',
    items: ['지혈 패드', '압박 붕대', '소독제'],
  },
  chronic: {
    title: '추천 키트: 기저질환 의심',
    items: ['혈압계', '혈당 측정 키트', '기본 구급 키트'],
  },
};

export default function DashboardPage() {
  const [selectedEvent, setSelectedEvent] = useState<string>('exhaustion');
  const [logs, setLogs] = useState<string[]>([]);
  const [isSending, setIsSending] = useState(false);

  const currentKit = KIT_PRESETS[selectedEvent];

  const handleDispatch = async () => {
    setIsSending(true);

    // TODO: 나중에 여기에서 /api/items 로 POST 요청 보낼 예정
    // await fetch('/api/items', { method: 'POST', body: JSON.stringify({...}) })

    const now = new Date().toLocaleTimeString();
    setLogs((prev) => [
      `[${now}] 로봇 파견 요청 - 유형: ${EVENT_OPTIONS.find(
        (e) => e.value === selectedEvent,
      )?.label}`,
      ...prev,
    ]);

    setIsSending(false);
  };

  return (
    <main className="min-h-screen bg-slate-950 text-slate-50 flex flex-col">
      {/* 상단 헤더 */}
      <header className="border-b border-slate-800 px-6 py-4 flex items-center justify-between">
        <h1 className="text-xl font-semibold">
          SSAFETY BOT · 관리자 대시보드
        </h1>
        <span className="text-xs text-slate-400">
          Login Section
        </span>
      </header>

      {/* 메인 레이아웃 */}
      <div className="flex-1 grid grid-cols-1 lg:grid-cols-3 gap-4 p-6">
        {/* 왼쪽: 쓰러짐 캡처 프레임 */}
        <section className="lg:col-span-2 bg-slate-900 rounded-2xl border border-slate-800 p-4 flex flex-col">
          <h2 className="text-sm font-medium text-slate-200 mb-3">
            쓰러짐 캡처 프레임
          </h2>
          <div className="flex-1 rounded-xl border border-dashed border-slate-700 flex items-center justify-center text-slate-500 text-sm">
            {/* TODO: 나중에 실제 이미지 캡처 썸네일로 교체 */}
            아직 이벤트가 없습니다. 쓰러짐이 감지되면 여기에 프레임이 표시됩니다.
          </div>
        </section>

        {/* 오른쪽: 상태 선택 + 키트 + 파견 */}
        <section className="bg-slate-900 rounded-2xl border border-slate-800 p-4 flex flex-col gap-4">
          {/* 상태 선택 */}
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
          </div>

          {/* 추천 키트 */}
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

          {/* 로봇 파견 버튼 */}
          <div className="mt-auto">
            <button
              onClick={handleDispatch}
              disabled={isSending}
              className="w-full rounded-lg bg-emerald-500 hover:bg-emerald-400 disabled:bg-emerald-700 disabled:cursor-wait text-slate-950 font-semibold py-2 text-sm transition"
            >
              {isSending ? '로봇 파견 중…' : '로봇 파견'}
            </button>
            <p className="mt-2 text-[11px] text-slate-500">
              * 클릭 시 Dobot 제어 서버로 응급 키트 정보와 환자 위치 정보를 JSON으로 전송할 예정입니다.
            </p>
          </div>
        </section>
      </div>

      {/* 하단 로그 */}
      <section className="border-t border-slate-800 px-6 py-3 bg-slate-950">
        <h2 className="text-xs font-medium text-slate-300 mb-1">
          이벤트 로그
        </h2>
        {logs.length === 0 ? (
          <p className="text-[11px] text-slate-500">
            아직 로봇 파견 로그가 없습니다.
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
