# Read-only catalog assembly

`mhseals_testing.catalog.assemble({owner_package: bundle}, required=...)` consumes
explicit core `DescriptorBundle` objects. Domain loaders receive or load their own
package resources; testing does not discover plugins or import ROS. Required
owners default to hardware and navigation. Missing bundles have explicit catalog
blockers, unexpected owners and any canonical/alias collisions refuse assembly.
Rows expose immutable defaults, default source references, descriptor revisions,
and blockers. Aliases resolve to the same row. Metadata cannot authorize writes.
Requested, effective, live, source-layer and B/S/T observations are absent.

`catalog_coverage.reconcile(catalog, resources, families, traces=..., dispositions=...)`
accepts only declared bounded inputs. Resources map `(owner, source_id)` to
`(current_revision, bytes)`; descriptor source hashes must match. Mapping leaves,
including empty mappings, are enumerated by YAML pointer. Sequence values remain
single fields, matching the descriptor type contract. A matching default reference
is descriptor-backed, which does not mean startup/live/behavior verified. Other
leaves are untraced. Families map `(owner, historical_audit_id)` to current source
revisions. Complete explicit trace mappings may mark a family descriptor-backed;
representative descendants alone cannot do so. Fixed/unsupported dispositions
need explicit reasons; all other families remain untraced named follow-ups.

The historical audit source manifests remain unchanged. Current revision strings
are supplied separately in report rows; they never overwrite historical evidence.
`markdown(rows)` renders the same reconciliation rows as read-only documentation.
This is a bounded inventory, not complete stack coverage or a writer interface.

Implementation validation uses the actual hardware package resource bundle and
an explicitly archived navigation revision. Integration approval still requires
the accepted navigation endpoint. Additional hardware, RTK, health, keymap,
profile, costmap, planner, perception and launch families need owner trace and
metadata follow-ups. No runtime adapter or manager is introduced here.

## Public CLI and TUI access

Configuration inventory is available without collecting boat health:
`boat_trial config catalog` prints the installed hardware, Nav2, MAVROS, lidar, odometry and localization
catalogs, plus the optional installed Astro run catalog. Use `--json -` for stdout
or `--json PATH` to save the same view. Missing or broken providers remain visible;
non-optional provider errors return exit status 2. This command exposes no mutation
verbs.

In the TUI, select the **Config** status row, press Enter, then select **Catalog**.
Up/Down reaches each package and leaf; Enter opens scrollable metadata and blocker
details; Esc returns. Watched file rows still apply their pending file on Enter.
Catalog defaults are package inventory, with source pointers/hashes, constraints,
consumers and coverage—not live values or verified device behavior. Without an
explicit compiler context, selection is unknown. Optional compiled values and their
origins remain separate and are labeled requested/compiled, never live.

Odometry/localization defaults remain raw installed package values. Their resolvable
`constraint_ref` IDs use the same Rule objects as validation; concrete schema rules
are in the generated [startup constraint guide](https://github.com/mhseals/mhseals_nav/blob/main/docs/startup-catalog-constraints.md),
also installed at `share/mhseals_nav/docs/startup-catalog-constraints.md`. Details
include this resource link; generic `numeric_bounds` stays unknown, and `units`
is not used for constraint prose. The guide includes enums, exact vector shape,
known bounds and unverified consumer distinctions. All rows remain read-only.
Fresh-start/restart-required describes startup consumption, not a runtime restart
capability; selected compiled values/origins remain separate from defaults and live state.
