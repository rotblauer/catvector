package main

import (
	"time"

	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
)

type CompositeFeature struct {
	*WrappedFeature
	Elements []*WrappedFeature
}

func (c *CompositeFeature) MustActivity() string {
	return c.WrappedFeature.MustActivity()
}

func (c *CompositeFeature) MustActivities() string {
	activities := ""
	for _, f := range c.Elements {
		activities += f.MustActivity() + " "
	}
	return activities

}

type WrappedFeature struct {
	*geojson.Feature
}

// MarshalJSON implements the json.Marshaler interface.
func (f *WrappedFeature) MarshalJSON() ([]byte, error) {
	return f.Feature.MarshalJSON()
}

// UnmarshalJSON implements the json.Unmarshaler interface.
func (f *WrappedFeature) UnmarshalJSON(data []byte) error {
	return f.Feature.UnmarshalJSON(data)
}

func (f *WrappedFeature) MustActivity() string {
	return f.Feature.Properties.MustString("Activity")
}

func (f *WrappedFeature) MustName() string {
	return f.Feature.Properties.MustString("Name")
}

func (f *WrappedFeature) MustUUID() string {
	return f.Feature.Properties.MustString("UUID")
}

func (f *WrappedFeature) MustAccuracy() float64 {
	return f.Feature.Properties.MustFloat64("Accuracy")
}

func (f *WrappedFeature) MustSpeed() float64 {
	return f.Feature.Properties.MustFloat64("Speed")
}

func (f *WrappedFeature) MustTime() time.Time {
	t := f.Feature.Properties.MustString("Time")
	out, err := time.Parse(time.RFC3339, t)
	if err != nil {
		panic(err)
	}
	return out
}

func (f *WrappedFeature) MustGetDistance() float64 {
	if f.Geometry.GeoJSONType() == "Point" {
		return 0
	}
	bound := f.Geometry.Bound()
	return geo.Distance(bound.Min, bound.Max)
}

func (f *WrappedFeature) MustGetLength() float64 {
	if f.Geometry.GeoJSONType() == "Point" {
		return 0
	}
	return geo.Length(f.Geometry)
}
