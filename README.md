Turns cat tracks into cat vectors -- geojson linestrings.

This project is primarily intended to develop a trip detection pipeline.
Trips are represented as linestrings, and non-trips (stops/visits/naps/pauses)
can be represented as points.
`cmdTripDetector` is the initial and, at the time of writing, the current state of the art.
It was derived largely from some whitepapers I found on trip detection algorithms, and
uses dwell time, dwell distance, and time between track points.
Now, I'd like to pursue an experiment with "knots" for pause detection.
How does Mike mark a down elk on the map? He walks a 10m circle around it, tying a knot in the line
so he can identify it easily on a map. So maybe tracks with intersection/s occurring within some dwell
time might be suggestive of "stops."

Use [cattracks-explorer](https://github.com/rotblauer/cattracks-explorer) to visualize the generated data.
- http://localhost:8080/public/?vector=http://localhost:3001/services/ia/valid/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/laps/tiles/{z}/{x}/{y}.pbf
- http://localhost:8080/public/?vector=http://localhost:3001/services/rye/valid/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/laps/tiles/{z}/{x}/{y}.pbf

20240920

I ran the pipeline on master.

The script halted after `ia`, probably due to some error along the way (probably buried in logs).
The results look quite good.
A few things to improve:
- a big long straight "walking" line connect Spokane with Williamsburg. Long straight lines are impossible.
- "Stationary" laps exist (eg RAGBRAI 2023)
- homes are still spike balls, but not terrrible. This might be unavoidable to some extent, since I do wander around homebases randomly.
- some bike laps are broken into too many segments (see twin lakes ride from the River)
- where is my train ride Brussels-Paris (and back)? spotty sections.
  - The missing train ride points exists in `valid`, but are not getting picked up by the trip detector.
    - urban canyon filtered?
- where are the flights? are these points getting filtered for accuracy/speed? in the prelim cleanup stage?

## Instructions

```sh
time env PARALLEL_JOBS=8 PARALLEL_BATCH_SIZE=100000 ./run.sh |& tee run.out
```

```sh
mbtileserver --port 3001 --cors '*' -d /home/ia/tdata/local/catvector/direct-master --verbose --enable-fs-watch
```
> [mbtileserver](https://github.com/rotblauer/mbtileserver)

```sh
cd cattracks-explorer && yarn install && yarn dev
echo 'Go here http://localhost:8080/public/?vector=http://localhost:3001/services/ia/laps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/valid/tiles/{z}/{x}/{y}.pbf'
```
> [cattracks-explorer](https://github.com/rotblauer/cattracks-explorer)


- [run.sh](run.sh) is the script to define the tracks-source and the target cat loop. It runs `gen.sh` and `tile.sh`. 
- [setup.sh](setup.sh) is the script to set up the environment for the pipeline. It defines environment variables and builds the Go project.
  You normally don't have to touch this.
- [gen.sh](gen.sh) is the tracks pipeline script, turning original source tracks into validated, filtered, trip-detected, simplified, stop-consolidated tracks in GeoJSON form.   
  It relies on these other CLI tools in addition to the local Go code. 
  - [`cattracks-names`](https://github.com/rotblauer/cattracks-names)
  - [`gfilter`](https://github.com/rotblauer/gfilter)
- [tile.sh](tile.sh) is the script to generate vector tiles from the generated geojson features.
  - [runtpl.sh](runtpl.sh) is the script to run `tippecanoe` with settings for LINESTRINGS.
  - [runtpp.sh](runtpp.sh) is the script to run `tippecanoe` with settings for POINTS (specifically "naps").


### RKalman bug
There was a bug where the regnull/Kalman filter would panic
with slice out of bounds on the LAT pre-computed (fast-for-lat-degrees-to-meters).
It was getting like 100 lat but list size 91. 
I seem to have fixed this by validating points before passing them to the filter,
and, IMPORTANTLY, asserting that all points MUST NOT have any 0-value coordinate.

There are a few of them. Most of the logged zero-value-coordinates cattracks
have `visit`s too. So I wonder if there's a bug there in the iOS. The `visit` JSON value in the point does have the full coordinates, though, so they're recoverable.

PS That might now have actually fixed it.... yea, nope.

---
