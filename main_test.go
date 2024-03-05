package main

import (
	"encoding/json"
	"image/color"
	"io"
	"log"
	"os"
	"path/filepath"
	"testing"

	sm "github.com/flopp/go-staticmaps"
	"github.com/fogleman/gg"
	"github.com/golang/geo/s2"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/planar"
	"github.com/paulmach/orb/simplify"
)

var testdataRootPath = filepath.Join(".", "testdata")
var testdataOutputRootPath = filepath.Join(testdataRootPath, "output")

func testdataReader(geojsonFile string) (io.Reader, error) {
	return os.Open(filepath.Join(testdataRootPath, geojsonFile))
}

func paintMapWriting(f *geojson.Feature, pathto string) error {
	ctx := sm.NewContext()
	ctx.SetSize(800, 800)

	switch g := f.Geometry.(type) {
	case orb.LineString:
		lls := []s2.LatLng{}
		for _, v := range g {
			lls = append(lls, s2.LatLngFromDegrees(v[1], v[0]))
		}
		v := sm.NewPath(lls, color.RGBA{0, 0, 255, 255}, 3)
		ctx.AddObject(v)
	}

	img, err := ctx.Render()
	if err != nil {
		return err
	}

	os.MkdirAll(filepath.Dir(pathto), 0777)
	if err := gg.SavePNG(pathto, img); err != nil {
		return err
	}
	return nil
}

func TestReadStreamToLineString(t *testing.T) {
	in, err := testdataReader("edge2.json")
	if err != nil {
		t.Fatal(err)
	}
	f, err := readStreamToLineString(in)
	if err != nil {
		t.Fatal(err)
	}

	// j, err := json.MarshalIndent(f, "", "  ")
	j, err := json.Marshal(f)
	if err != nil {
		t.Fatal(err)
	}
	log.Println(string(j))

	if err := paintMapWriting(f, filepath.Join(testdataOutputRootPath, "edge_original.png")); err != nil {
		t.Fatal(err)
	}

	lsSimpler := f.Geometry.(orb.LineString).Clone()

	// Simplify
	threshold := planar.Distance(orb.Point{0, 0}, orb.Point{0, 0.00006})
	log.Printf("threshold: %f", threshold)
	simplifier := simplify.DouglasPeucker(float64(6) * earthCircumferenceDegreesPerMeter)
	lsSimpler = simplifier.Simplify(lsSimpler).(orb.LineString)

	// j, err := json.MarshalIndent(f, "", "  ")
	j, err = json.Marshal(lsSimpler)
	if err != nil {
		t.Fatal(err)
	}
	log.Println(string(j))

	distance := planar.Distance(f.Geometry.Bound().Min, f.Geometry.Bound().Max)
	t.Logf("distance: %f", distance)
	metersQ := earthCircumference * distance
	t.Logf("metersQ: %f", metersQ)

	if err := paintMapWriting(geojson.NewFeature(lsSimpler), filepath.Join(testdataOutputRootPath, "edge_simplified.png")); err != nil {
		t.Fatal(err)
	}

	// KFilter

	lsKalman := f.Geometry.(orb.LineString).Clone()
	lsKalmanF := geojson.NewFeature(lsKalman)
	for k, v := range f.Properties {
		lsKalmanF.Properties[k] = v
	}
	if err := modifyLineStringKalman(lsKalmanF); err != nil {
		t.Fatal(err)
	}

	if err := paintMapWriting(lsKalmanF, filepath.Join(testdataOutputRootPath, "edge_kalman.png")); err != nil {
		t.Fatal(err)
	}

	// KFilter - github.com/Regnull - geokalmanfilter

	lsKalmanRegnull := f.Geometry.(orb.LineString).Clone()
	lsKalmanRegnullF := geojson.NewFeature(lsKalmanRegnull)
	for k, v := range f.Properties {
		lsKalmanRegnullF.Properties[k] = v
	}
	if err := modifyLineStringKalmanRegnull(lsKalmanRegnullF); err != nil {
		t.Fatal(err)
	}

	if err := paintMapWriting(lsKalmanRegnullF, filepath.Join(testdataOutputRootPath, "edge_kalman_regnull.png")); err != nil {
		t.Fatal(err)
	}

}
