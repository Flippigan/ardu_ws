#!/usr/bin/env bash
# Iteratively implement the collision-avoidance plan, one step per Claude session.
# Bash/`claude -p` equivalent of run_collision_avoidance_loop.py — no SDK needed.
set -u

PLAN="/home/finn/Documents/ardu_ws/src/formation_control/.claude/Plans/2026-07-23-collision-avoidance-integration.md"
PROGRESS="/home/finn/Documents/ardu_ws/src/formation_control/.claude/Progress/2026-07-23-collision-avoidance-integration-progress.md"
MAX_ITERATIONS=40
SENTINEL="PLAN_COMPLETE"

PROMPT="/superpowers:using-superpowers implement '$PLAN' using subagent driven development.
All subagents must be experts in their respective fields.
You MUST record their progress in '$PROGRESS'.

IMPORTANT session rules:
1. First read '$PROGRESS' to see what is already done.
2. Complete exactly ONE step/task of the plan this session - no more.
3. After the step is done and verified, update '$PROGRESS' with what was
   completed, any decisions made, and what the NEXT step is.
4. Then STOP. Do not start the next step.
5. If every step of the plan is already complete and verified, do nothing and
   output exactly: $SENTINEL"

cd /home/finn/Documents/ardu_ws || exit 1

SESSION=""
for i in $(seq 1 "$MAX_ITERATIONS"); do
  echo ""
  echo "============================================================"
  echo "ITERATION $i (session: ${SESSION:-new})"
  echo "============================================================"

  if [ -z "$SESSION" ]; then
    OUT=$(claude -p "$PROMPT" --output-format json --permission-mode bypassPermissions)
  else
    OUT=$(claude -p "$PROMPT" --resume "$SESSION" --output-format json --permission-mode bypassPermissions)
  fi

  if [ -z "$OUT" ] || ! echo "$OUT" | python3 -c "import json,sys; json.load(sys.stdin)" 2>/dev/null; then
    echo "Iteration $i produced no valid JSON - starting fresh session next round"
    SESSION=""
    continue
  fi

  json_field() { echo "$OUT" | python3 -c "import json,sys; print(json.load(sys.stdin).get('$1', '$2'))"; }
  SESSION=$(json_field session_id "")
  RESULT=$(json_field result "")
  IS_ERROR=$(json_field is_error false | tr 'A-Z' 'a-z')
  COST=$(json_field total_cost_usd 0)

  echo "$RESULT"
  echo "--- session $SESSION finished: \$$COST, is_error=$IS_ERROR ---"

  if [ "$IS_ERROR" = "true" ]; then
    echo "Iteration $i errored - starting fresh session next round"
    SESSION=""
    continue
  fi

  if echo "$RESULT" | grep -q "$SENTINEL"; then
    echo ""
    echo "Plan complete after $i iteration(s)."
    exit 0
  fi
done

echo ""
echo "Stopped: hit MAX_ITERATIONS ($MAX_ITERATIONS) without $SENTINEL."
exit 1
