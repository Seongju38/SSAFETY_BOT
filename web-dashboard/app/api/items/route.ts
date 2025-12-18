// POST /api/items
import { NextResponse } from "next/server";

export async function POST(request: Request) {
  const body = await request.json();

  // 실제 Dobot 제어 서버(Python)로 전달
  // await fetch("http://localhost:8000/dispatch", {
  //   method: "POST",
  //   headers: { "Content-Type": "application/json" },
  //   body: JSON.stringify(body),
  // });

  console.log("📦 [Mock Robot Dispatch] request body:", body);

  // 진짜 로봇 서버에 보내는 대신, 여기서 잠깐 기다렸다가 가짜 응답을 돌려줌
  await new Promise((resolve) => setTimeout(resolve, 500));

  const mockJobId =
    "MOCK-JOB-" +
    Math.floor(Math.random() * 100000)
      .toString()
      .padStart(5, "0");

  return NextResponse.json({
    ok: true,
    jobId: mockJobId,
    received: body, // 디버깅용으로 받은 데이터도 같이 돌려줌
  });
}
