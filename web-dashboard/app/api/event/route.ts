// POST /api/event
import { NextResponse } from "next/server";

export async function POST(request: Request) {
  const data = await request.json();

  // 여기서 로봇 서버(Python)로 전달할 수도 있음
  // await fetch("http://localhost:8000/event", { ... });

  console.log("📩 Event received:", data);

  return NextResponse.json({ ok: true });
}
