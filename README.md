# catvector

This project is primarily intended to develop a trip detection pipeline,
where: trips ("laps") are represented as linestrings and non-trips ("naps") can be represented as points.

How to differentiate laps from naps?
What does this even mean?
Does a lap begin when the cat ties their shoes? Upon leaving the front door?
Both Android and iOS clients offer an "Activity: (Unknown|Stationary|Walking|Running|Biking|Driving)" annotation.
Should laps differentiate themselves by activity? (Should a bicycle ride home from a run count as two laps or one?)
 
Is pacing between the chair and the stove a "lap"?
Laps between the bakery and produce departments in a large grocery store?  
Will airplane trips be represented as laps?
Are laps essentially `Speed > 0`, and naps then `Speed == 0`?   
Should laps end, or should naps begin?

Should they be exclusive? (They might use weighting or confidence scores instead; view could be decided by client.)
... Or, Should we avoid the binary categorization and do scoring or tagging instead?

This project so far is designed generally as a series of point skips and mutations
which is applied to an io.Writer stream of newline-delimited points (properly: cat tracks).
Sloppily, so far, some of these steps use backward-facing sets of data (cached tracks) - these are "stateful" because they need to remember "what happened",
some use forward-facing data as well (becoming "eventers" that rely on non-serial reads/processes; they need to know what happens before _and_ after some cursor point in time and space) - like 
`wangUrbanCanyonFilterStream`; other functions are atomic - they only require at a single cat track to operate on it.
It might be good to structurally differentiate these kinds of functions,
since speed, correctness, and elegance are all being explored here. 

This design is tailored to data formats that are quickly serially readable and writing, but not necessarily/easily indexable or queryable.
For CatTracks this usually means newline-delimited, gzipped files containing lists of GPS-annotated GeoJSON-encoded `Point`-type `Feature`s.
These points may have many `properties`, like `"Accuracy": 5`, `"Speed": 1.42"`, or `"Name": "isaac"`,
which must be handled with care owing to their legacy variations over time and clients.

This project uses `zcat` widely.  

The venerable `tippecanoe` is used finally to produce `.mbtiles` vector tile databases.
It is important to note that tippecanoe is responsible for VERY IMPORTANT decisions about ultimate feature inclusion, mutations,
simplifications, and other client-facing data. Server-side rendering here, people.

Vector tiles can then be served to the client with a tile server.
The client will then also be responsible for important decisions about what to show and how to show it on the map.
Cat Tracks' state of the art uses the _MapLibre GL JS_ web client library regularly.

Why a pipeline? What does the pipeline look like?

Read
Validate
Clean
Analyze
Filter
Transform
Tee

A "cleaning" step drops data from the pipeline, which can decrease resource demands like time and memory.
Invalid data can be readily cleaned.
A "validate" step asserts assumptions about the data.




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

http://localhost:8080/public/?vector=http://localhost:3001/services/ia/laps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/naps/tiles/{z}/{x}/{y}.pbf
http://localhost:8080/public/?vector=http://localhost:3001/services/rye/laps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/naps/tiles/{z}/{x}/{y}.pbf


---

20241018

- [ ] All `populate/` cattracks should organize to `cat/` subdirs. 
- [ ] (Refactor catm2 to organize subdirectories for `cat/`.)
  - Level 23
  - Level 16
- [ ] Add dedicated `cat/snaps` layers.
- [ ] Put laps into a Postgres+PostGIS DB? 

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
