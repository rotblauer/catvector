Turns cat tracks into cat vectors, ie. linestrings.


There was a bug where the regnull/Kalman filter would panic
with slice out of bounds on the LAT pre-computed (fast-for-lat-degrees-to-meters).
It was getting like 100 lat but list size 91. 
I seem to have fixed this by validating points before passing them to the filter,
and, IMPORTANTLY, asserting that all points MUST NOT have any 0-value coordinate.

There are a few of them. Most of the logged zero-value-coordinates cattracks
have `visit`s too. So I wonder if there's a bug there in the iOS. The `visit` JSON value in the point does have the full coordinates, though, so they're recoverable.

PS That might now have actually fixed it....
