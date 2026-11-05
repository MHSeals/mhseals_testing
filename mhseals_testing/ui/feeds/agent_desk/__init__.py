"""The agent desk by responsibility (ui/feeds/agent.py re-exports it; desk.py is the glue and its contract).

common.py holds the desk's constants: the ops it treats as motion, urgent or changing, its timings and the
operator hints of each cancel cause.

The gates, in the order a request meets them; the first that fails answers it (test_agent_desk.py's gate table):
1. channel.on_req: an id already answered is dropped (a replayed line never acts twice).
2. channel.on_req: the deadline is clamped to the request's own stamp plus request_s (a replay gains no time).
3. channel.on_req: a motion request stamped before this TUI started is expired (nothing from a past session moves).
4. channel.on_req: a request past its deadline is expired (its CLI already gave up).
5. channel.on_req: paused for a changing op while the operator state is not normal (before any op's own checks).
6. channel.on_req: bad_op for an op with no handler; then the op's handler runs.
For run and goal, queue.refusal, at enqueue and again as queue.start_next starts it (the world changed meanwhile):
7. paused (a request queued before the pause), 8. motion_observe (Agent motion), 9. agents_block (Agents),
10. the claim (claim_state: revoked, expired, not_holder, ...), 11. no_mux (no muxed profile or sim),
12. overridden (a human drives), 13. run only: the agent.yaml run: allowlist and its args, sim_arm, the settings.
For claim, lease.do_claim: 7. no_owner (no arbiter), 8. blocked (Agents block),
9. ros/authority Arbiter._claim: busy (one holder at a time), which the desk names duplicate_who for the same --who.
"""
