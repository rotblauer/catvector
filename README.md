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

---

20241005

Real cats will push 100 points at a time every ~2minutes.
Is the trip detector ready for an intermittent stream of tracks?
Should the TD use a persistent state to track cats?
Or should it get fed redundant batches, reprocessing them, and using
an edge/devop/master pattern? 

The Trip Detector and Stop Consolidator both accept streams of points read over stdin,
and both output streams of (Geo)JSON to stdout. The output stream can be gzip-appended
to any existing compressed gzip file (probably containing a list of GeoJSON features).



20241004

```sh
  echo >&2 "Processing category: ${CAT_ONE}, batch: ${batch_id}"
  ${BUILD_TARGET} validate \
    | cattracks-names modify-json --modify.get='properties.Name' --modify.set='properties.Name' \
    | gfilter --match-all '#(properties.Name=='"${CAT_ONE}"')' \
    | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/valid/batch-${batch_id}.json.gz" \
    | ${BUILD_TARGET} --urban-canyon-distance=200 preprocess \
    | ${BUILD_TARGET} --dwell-interval=120s --dwell-distance=15 --trip-start-interval=30s --speed-threshold=0.5 trip-detector \
    | tee >( \
      gfilter --ignore-invalid --match-all '#(properties.IsTrip==true)' \
        | ${BUILD_TARGET} --dwell-interval=120s points-to-linestrings \
        | ${BUILD_TARGET} --threshold=0.00008 douglas-peucker \
        | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/linestrings/batch-${batch_id}.json.gz" \
    ) \
    | tee >( \
      gfilter --ignore-invalid --match-all '#(properties.IsTrip==false),#(properties.MotionStateReason!="reset")' \
        | ${BUILD_TARGET} --dwell-interval=120s --dwell-distance=100 consolidate-stops \
        | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/points/batch-${batch_id}.json.gz" \
    )
```

Above is a copy of the logical heart of the `gen.sh` pipeline.

- Contiguous naps and laps?
I am realizing now that a limitation of structurally forking laps and naps logic and data
is that NAPS and LAPS are not respective of each other. Intuitively some cat's lap must begin from exactly where it finished napping, and must end where it starts its next nap; laps and naps should be contiguous. Currently, naps are "floating" points usually _near_ to starts and stops of lap starts/stops, but not contiguous with them. This will also mean difficulty or impossible temporal indexing of laps and naps relative to each other. "Tell me about the naps before and after some lap" may be a hard question. Can I MERGE the two data sets? It might be nice to be able to tell the "story" in a timeline-style way; nap, lap, another nap... etc. On the other hand, we can expect the timestamps of both naps and laps to be consistent with and respective of each other; so a front-end `sort` on `Time` should be able to merge the resources. 

Another feature: indexing naps (and laps?). To be able to say: this cat has napped here 22 times; here are those naps.
Laps, too; like Strava segments -- these will be harder. But it would be awesome to be able to look at Rye's
morning runs as laps of a common track.   

...

What if NAPS were consolidated as areas (ie S2 cells) of X size.
How would each naps get aggregated to the larger napping place?
- total nap time spent there
- latest nap time, first nap time


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
