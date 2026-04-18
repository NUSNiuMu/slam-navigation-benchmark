# Documentation Guide

This top-level `docs/` directory is the curated reading entry for the repository.

It is meant to make the project easier to navigate without changing the original code layout inside the ROS workspaces.

## Contents

- `notion_done/`
  - Synced and cleaned copies of project notes whose status is `Done` in the Notion FYP database.
- `reports/`
  - Final report entry and generated PDF.
- `waypoints/`
  - Example waypoint YAML files preserved for evaluation reference.

## Relationship to the Original Workspace Files

The original project files remain in their workspace locations, especially under:

- `ws_livox/src/mid360_navigation/`
- `fastlio2_ws/src/FAST_LIO/`
- `fastlio2_ws/src/Point-LIO/`
- `fasterlio_ws/src/faster-lio/`

This curated `docs/` layer is intentionally additive. It improves readability on GitHub without breaking scripts, launch files, or report paths that still depend on the original workspace structure.
