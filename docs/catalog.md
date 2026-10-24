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
