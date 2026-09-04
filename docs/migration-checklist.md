# AURORA Migration Checklist

## Progress

- [x] Create the initial AURORA structure.
- [x] Create introductory documentation.
- [x] Create a high-level LIMB overview.
- [ ] Create a detailed LIMB inventory.
- [ ] Confirm the active arm-control software.
- [ ] Confirm the firmware used by each controller.
- [ ] Document the existing CAN protocol.
- [ ] Identify communication conflicts.
- [ ] Identify working and broken hardware.
- [ ] Identify duplicate and generated files.
- [ ] Select files for migration.
- [ ] Migrate one subsystem at a time.
- [ ] Test each migrated subsystem.
- [ ] Add simulation and safety validation.
- [ ] Verify the complete system.

## Rules for later migration

For each candidate, record its original LIMB path, purpose, dependencies, hardware relationship, provenance, apparent status, safety impact, duplicates, destination, and validation plan. Mark conclusions as unconfirmed until supported by source review or test evidence.

Only approved source material should be copied. Generated files, caches, downloads, recordings, datasets, manufacturing exports, and backups require an explicit reason and retention decision. Preserve traceability even when a file is renamed or reorganized.

Migration should follow dependencies: establish the as-built hardware and protocol record first, then shared firmware and communication boundaries, then one controller or host subsystem at a time. Research features should not be placed on the restoration-critical path until the base arm is reliable.

## Step 2 exit criteria

Before migration starts, the team should have a reviewable inventory, a candidate active-system map, a controller/firmware matrix, a reconciled draft CAN specification, a hardware condition list, and a list of duplicate/generated artifacts to exclude or retain. All uncertain points should have an owner or a proposed way to confirm them.
