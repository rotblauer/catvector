Turns cat tracks into cat vectors, ie. linestrings.

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
