#!/usr/bin/env bash
# Runs every tools/bench.ps1 rung at workers 1 and 8 with per-tick hash logs.
#   ladder.sh <lpf_bench> <outdir> [runner prefix, e.g. box64 or arch -x86_64]
# Writes <outdir>/hashes.txt ("rung workers hash solverHash exit"), <outdir>/ticks/<rung>.w<N>.txt and the bench output.
bench="$1"
out="$2"
shift 2
mkdir -p "$out/ticks"
: > "$out/hashes.txt"
for spec in walls:walls:12 town:town:12 pile:pile:12 lumber:lumber:12 tower:tower:12 ruins:ruins:12 yard:yard:12 \
	keep:keep:12 barrage:town:3 siege:keep:4 track:track:30 mech:mech:30; do
	rung="${spec%%:*}"
	rest="${spec#*:}"
	scene="${rest%%:*}"
	period="${rest#*:}"
	start=$(date +%s)
	"$@" "$bench" --scene "$scene" --workers 1,8 --ticks 600 --period "$period" --json "$out/$rung.json" \
		--hash-log "$out/ticks/$rung" > "$out/$rung.txt" 2>&1
	code=$?
	secs=$(( $(date +%s) - start ))
	if [ -f "$out/$rung.json" ]; then
		grep -oE '"workers": [0-9]+|"hash": "[0-9a-f]+"|"solverHash": "[0-9a-f]+"' "$out/$rung.json" | sed 's/.*: //; s/"//g' |
			paste -d' ' - - - | while read -r w h s; do echo "$rung $w $h $s exit=$code secs=$secs"; done >> "$out/hashes.txt"
	else
		echo "$rung - - - exit=$code secs=$secs" >> "$out/hashes.txt"
	fi
	echo "$rung: exit $code in ${secs}s"
done
cat "$out/hashes.txt"
