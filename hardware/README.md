# Hardware

## Purpose

This folder will hold the reviewed, source-of-truth description of AURORA's physical system: BOM, PCB files, schematics, wiring, repairs, and selected CAD.

## What belongs here

- An as-built hardware inventory and condition/repair records.
- Editable PCB and schematic sources tied to known board revisions.
- Wiring, connector, power, protection, and controller-placement documentation.
- A maintained BOM and selected native CAD or fabrication sources needed for the actual arm.

## What does not belong here

- Host software or firmware.
- Unexplained manufacturing exports, automatic backups, caches, logs, or redundant mesh copies.
- CAD used only by an exploratory simulation when it can be generated or referenced from a canonical model.
- Files whose origin, units, revision, or relationship to the physical arm is unknown without an explicit quarantine decision.

## Possible LIMB migration

LIMB contains an Excel BOM, KiCad projects for arm modules and a CAN transceiver, EMG electronics, Gerber/drill/CNC exports, Multisim/Ultiboard material, photographs, many STL meshes, STEP models, and URDF descriptions. Candidates must be matched to installed hardware and source revisions; backups, exports, and repeated meshes should not be copied automatically.

