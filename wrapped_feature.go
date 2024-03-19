package main

import (
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
)

type CompositeFeature struct {
	*TrackGeoJSON
	Elements []*TrackGeoJSON
}

func NewCompositeFeature(f *TrackGeoJSON) *CompositeFeature {
	return &CompositeFeature{
		TrackGeoJSON: f,
		Elements:     []*TrackGeoJSON{f},
	}
}

func (f *CompositeFeature) AddFeature(feat *TrackGeoJSON) {
	f.Elements = append(f.Elements, feat)
}

func (f *CompositeFeature) MustGetDistance() float64 {
	var out float64
	for _, el := range f.Elements {
		out += el.MustGetDistance()
	}
	return out
}

func (f *CompositeFeature) MustGetLength() float64 {
	var out float64
	for _, el := range f.Elements {
		out += el.MustGetLength()
	}
	return out
}

type TracksGeoJSON []*TrackGeoJSON

func (f TracksGeoJSON) Len() int {
	return len(f)
}

func (f TracksGeoJSON) ForEach(fn func(*TrackGeoJSON)) {
	for _, el := range f {
		fn(el)
	}
}

func (f TracksGeoJSON) Points() []orb.Point {
	out := make([]orb.Point, len(f))
	for i, el := range f {
		out[i] = el.Geometry.(orb.Point)
	}
	return out
}

func (f TracksGeoJSON) Timespan() time.Duration {
	if len(f) == 0 {
		return 0
	}
	return f[len(f)-1].MustGetTime().Sub(f[0].MustGetTime())
}

func (f TracksGeoJSON) ReportedSpeeds() []float64 {
	out := make([]float64, len(f))
	for i, el := range f {
		out[i] = el.MustGetSpeed()
	}
	return out
}

func (f TracksGeoJSON) CalculatedSpeeds() []float64 {
	out := make([]float64, len(f))
	for i, el := range f {
		if i == 0 {
			out[i] = el.MustGetSpeed()
			continue
		}
		dist := geo.Distance(f[i-1].Geometry.(orb.Point), el.Geometry.(orb.Point))
		dur := el.MustGetTime().Sub(f[i-1].MustGetTime()).Seconds()
		out[i] = dist / dur
	}
	return out
}

type TrackGeoJSON struct {
	*geojson.Feature
}

func (f *TrackGeoJSON) ToFeature() *geojson.Feature {
	return f.Feature
}

// MarshalJSON implements the json.Marshaler interface.
func (f *TrackGeoJSON) MarshalJSON() ([]byte, error) {
	return f.Feature.MarshalJSON()
}

// UnmarshalJSON implements the json.Unmarshaler interface.
func (f *TrackGeoJSON) UnmarshalJSON(data []byte) error {
	return f.Feature.UnmarshalJSON(data)
}

func (f *TrackGeoJSON) MustGetActivity() string {
	return f.Feature.Properties.MustString("Activity")
}

func (f *TrackGeoJSON) MustGetName() string {
	return f.Feature.Properties.MustString("Name")
}

func (f *TrackGeoJSON) MustGetUUID() string {
	return f.Feature.Properties.MustString("UUID")
}

func (f *TrackGeoJSON) MustGetAccuracy() float64 {
	return f.Feature.Properties.MustFloat64("Accuracy")
}

func (f *TrackGeoJSON) MustGetSpeed() float64 {
	return f.Feature.Properties.MustFloat64("Speed")
}

func (f *TrackGeoJSON) MustGetTime() time.Time {
	t := f.Feature.Properties.MustString("Time")
	out, err := time.Parse(time.RFC3339, t)
	if err != nil {
		panic(err)
	}
	return out
}

func (f *TrackGeoJSON) After(t time.Time) bool {
	return f.MustGetTime().After(t)
}

func (f *TrackGeoJSON) MustGetDistance() float64 {
	if f.Geometry.GeoJSONType() == "Point" {
		return 0
	}
	bound := f.Geometry.Bound()
	return geo.Distance(bound.Min, bound.Max)
}

func (f *TrackGeoJSON) MustGetLength() float64 {
	if f.Geometry.GeoJSONType() == "Point" {
		return 0
	}
	return geo.Length(f.Geometry)
}
