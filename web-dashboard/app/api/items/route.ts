// POST /api/items
import { NextResponse } from "next/server";

export async function POST(request: Request) {
  const data = await request.json();

  // 실제 Dobot 제어 서버(Python)로 전달
  // await fetch("http://localhost:8000/dispatch", {
  //   method: "POST",
  //   headers: { "Content-Type": "application/json" },
  //   body: JSON.stringify(data),
  // });

  console.log("📦 Dispatch request:", data);

  return NextResponse.json({ ok: true });
}
