#!/bin/bash

#===============================================================================
# clean_mission_data.sh
#===============================================================================
# PURPOSE:
#   Deletes accumulated run folders and build logs from the mission_database
#   package directory.
#
#   Removes from database/:
#     Run folders named YYYY-MM-DD_HH-MM-SS/ and everything inside them
#     (*.db, *.db-shm, *.db-wal).  Any stray flat *.db files left over from
#     before the run-folder layout was adopted are also removed.
#
#   Removes from log/:
#     All build_YYYY-MM-DD_HH-MM-SS/ directories and the symlinks
#     (latest, latest_build) that point to them.
#     COLCON_IGNORE is preserved -- it tells colcon not to index the log dir.
#
# USAGE:
#   ./clean_mission_data.sh            # dry run -- shows what would be deleted
#   ./clean_mission_data.sh --confirm  # actually deletes
#
# SAFETY:
#   Dry-run mode is the default.  Nothing is deleted unless --confirm is passed.
#   The script resolves its own location so it works correctly regardless of
#   where you call it from.
#===============================================================================

set -euo pipefail

#===============================================================================
# RESOLVE PACKAGE ROOT
# Always operate relative to the directory this script lives in, which should
# be the mission_database package root.
#===============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DB_DIR="$SCRIPT_DIR/database"
LOG_DIR="$SCRIPT_DIR/log"

#===============================================================================
# PARSE ARGUMENTS
#===============================================================================

DRY_RUN=true
for arg in "$@"; do
    case "$arg" in
        --confirm) DRY_RUN=false ;;
        --help|-h)
            echo "Usage: $0 [--confirm]"
            echo "  (no args)   Dry run -- print what would be deleted"
            echo "  --confirm   Actually delete"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--confirm]"
            exit 1
            ;;
    esac
done

#===============================================================================
# HELPERS
#===============================================================================

print_header() { echo -e "\n\033[1;34m$1\033[0m"; }
print_item()   { echo -e "  \033[0;33m$1\033[0m"; }
print_none()   { echo -e "  \033[0;90m(none)\033[0m"; }
print_ok()     { echo -e "\033[1;32m$1\033[0m"; }
print_dry()    { echo -e "\033[1;33m[DRY RUN]\033[0m $1"; }

# Count the .db files inside a run folder for the display label.
count_dbs_in_folder() {
    local dir="$1"
    find "$dir" -maxdepth 1 \( -name "*.db" -o -name "*.db-shm" -o -name "*.db-wal" \) 2>/dev/null | wc -l
}

delete() {
    local target="$1"
    if [ "$DRY_RUN" = true ]; then
        print_item "would delete: $target"
    else
        print_item "deleting: $target"
        rm -rf "$target"
    fi
}

#===============================================================================
# PREAMBLE
#===============================================================================

echo "========================================"
echo " mission_database cleanup"
echo "========================================"
echo " Package root : $SCRIPT_DIR"
echo " Mode         : $([ "$DRY_RUN" = true ] && echo "DRY RUN (pass --confirm to delete)" || echo "LIVE -- files will be deleted")"
echo "========================================"

#===============================================================================
# DATABASES
#===============================================================================

print_header "Databases ($DB_DIR)"

if [ ! -d "$DB_DIR" ]; then
    echo "  directory does not exist -- skipping"
else
    DB_TARGETS=()

    # Run folders (named YYYY-MM-DD_HH-MM-SS).
    # Each contains {robot_name}.db (+ optional .db-shm/.db-wal sidecars).
    while IFS= read -r -d '' d; do
        DB_TARGETS+=("$d")
    done < <(find "$DB_DIR" -maxdepth 1 -type d \
               -regextype posix-extended \
               -regex '.*/[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}' \
               -print0 2>/dev/null | sort -z)

    # Stray flat *.db files at the top level (legacy layout, pre-run-folder).
    while IFS= read -r -d '' f; do
        DB_TARGETS+=("$f")
    done < <(find "$DB_DIR" -maxdepth 1 \
               \( -name "*.db" -o -name "*.db-shm" -o -name "*.db-wal" \) \
               -print0 2>/dev/null | sort -z)

    if [ ${#DB_TARGETS[@]} -eq 0 ]; then
        print_none
    else
        for t in "${DB_TARGETS[@]}"; do
            if [ -d "$t" ]; then
                n=$(count_dbs_in_folder "$t")
                delete "$t  ($n DB file(s) inside)"
            else
                delete "$t"
            fi
        done
        echo "  --- ${#DB_TARGETS[@]} item(s) total"
    fi
fi

#===============================================================================
# LOGS
#===============================================================================

print_header "Build logs ($LOG_DIR)"

if [ ! -d "$LOG_DIR" ]; then
    echo "  directory does not exist -- skipping"
else
    LOG_TARGETS=()

    # Build directories (named build_YYYY-MM-DD_HH-MM-SS).
    while IFS= read -r -d '' d; do
        LOG_TARGETS+=("$d")
    done < <(find "$LOG_DIR" -maxdepth 1 -type d -name "build_*" -print0 2>/dev/null | sort -z)

    # Symlinks (latest, latest_build -- but NOT COLCON_IGNORE).
    while IFS= read -r -d '' l; do
        LOG_TARGETS+=("$l")
    done < <(find "$LOG_DIR" -maxdepth 1 -type l -print0 2>/dev/null | sort -z)

    if [ ${#LOG_TARGETS[@]} -eq 0 ]; then
        print_none
    else
        for t in "${LOG_TARGETS[@]}"; do
            delete "$t"
        done
        echo "  --- ${#LOG_TARGETS[@]} item(s) total"
    fi

    # Confirm COLCON_IGNORE is preserved.
    if [ -f "$LOG_DIR/COLCON_IGNORE" ]; then
        echo -e "  \033[0;32m✓ COLCON_IGNORE preserved\033[0m"
    fi
fi

#===============================================================================
# SUMMARY
#===============================================================================

echo ""
if [ "$DRY_RUN" = true ]; then
    print_dry "Nothing deleted. Re-run with --confirm to actually clean."
else
    print_ok "Done."
fi
echo ""
