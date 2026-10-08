#!/usr/bin/env python3
"""Iteratively implement the collision-avoidance plan, one step per Claude session.

Each iteration runs one Claude Code session that is instructed to complete exactly
ONE step of the plan, record progress, and stop. The next iteration resumes the
same session (--resume under the hood) so it picks up exactly where it left off.
The loop ends when Claude prints the PLAN_COMPLETE sentinel or MAX_ITERATIONS hits.

Usage:  python3 run_collision_avoidance_loop.py
"""

import os
import sys

import anyio

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ResultMessage,
    SystemMessage,
    TextBlock,
    ToolUseBlock,
    query,
)

# Never let an exported API key hijack billing away from the Max subscription.
os.environ.pop("ANTHROPIC_API_KEY", None)

WORKSPACE = "/home/finn/Documents/ardu_ws"
PLAN = f"{WORKSPACE}/src/formation_control/.claude/Plans/2026-07-23-collision-avoidance-integration.md"
PROGRESS = f"{WORKSPACE}/src/formation_control/.claude/Progress/2026-07-23-collision-avoidance-integration-progress.md"

SENTINEL = "PLAN_COMPLETE"
MAX_ITERATIONS = 40

# Model for every session: full ID ("claude-fable-5") or alias ("opus", "sonnet").
MODEL = "claude-opus-4-8"

# Headless sessions do NOT auto-load installed plugins — superpowers must be
# passed explicitly or /superpowers:... comes back "Unknown command".
SUPERPOWERS = "/home/finn/.claude/plugins/cache/claude-plugins-official/superpowers/6.1.1"

PROMPT = f"""/superpowers:using-superpowers implement '{PLAN}' using subagent driven development.
All subagents must be experts in their respective fields.
You MUST record their progress in '{PROGRESS}'.

IMPORTANT session rules:
1. First read '{PROGRESS}' to see what is already done.
2. Complete exactly ONE step/task of the plan this session — no more.
3. After the step is done and verified, update '{PROGRESS}' with what was
   completed, any decisions made, and what the NEXT step is.
4. Then STOP. Do not start the next step.
5. If every step of the plan is already complete and verified, do nothing and
   output exactly: {SENTINEL}
"""


async def run_iteration(session_id: str | None) -> tuple[str | None, str]:
    """Run one session (resuming if session_id given). Returns (session_id, final_text)."""
    options = ClaudeAgentOptions(
        cwd=WORKSPACE,
        permission_mode="bypassPermissions",
        model=MODEL,
        # Load settings so project CLAUDE.md, agents, and skills are available.
        setting_sources=["user", "project", "local"],
        plugins=[{"type": "local", "path": SUPERPOWERS}],
        resume=session_id,
    )

    final_text = ""
    async for message in query(prompt=PROMPT, options=options):
        if isinstance(message, SystemMessage) and message.subtype == "init":
            api_key_source = message.data.get("apiKeySource")
            if api_key_source not in (None, "none"):
                sys.exit(
                    f"ABORT: session is using an API key (source: {api_key_source}) "
                    "instead of the Max subscription. Unset the key and rerun."
                )
        if isinstance(message, AssistantMessage):
            for block in message.content:
                if isinstance(block, TextBlock):
                    print(block.text)
                    final_text = block.text
                elif isinstance(block, ToolUseBlock):
                    args = str(block.input)
                    print(f"  [tool] {block.name}: {args[:200]}")
        elif isinstance(message, ResultMessage):
            session_id = message.session_id
            print(
                f"\n--- session {session_id} finished: "
                f"{message.num_turns} turns, ${message.total_cost_usd or 0:.4f}, "
                f"is_error={message.is_error} ---"
            )
            if message.result:
                final_text = message.result

    return session_id, final_text


async def main() -> None:
    session_id: str | None = None
    for i in range(1, MAX_ITERATIONS + 1):
        print(f"\n{'=' * 60}\nITERATION {i} (resume={session_id})\n{'=' * 60}")
        try:
            session_id, final_text = await run_iteration(session_id)
        except Exception as exc:  # noqa: BLE001 - keep the loop alive across transient failures
            print(f"Iteration {i} crashed: {exc!r} — starting fresh session next round")
            session_id = None
            continue

        if SENTINEL in final_text:
            print(f"\nPlan complete after {i} iteration(s).")
            return

    print(f"\nStopped: hit MAX_ITERATIONS ({MAX_ITERATIONS}) without {SENTINEL}.")


if __name__ == "__main__":
    anyio.run(main)
