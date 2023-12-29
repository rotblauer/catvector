package main

import (
	"bufio"
	"errors"
	"io"
	"log"
	"math"
	"os"
	"time"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

const (
	earthRadius                       = 6378137.0 // meters
	earthCircumference                = math.Pi * earthRadius * 2
	earthCircumferenceMetersPerDegree = earthCircumference / 360
	earthCircumferenceDegreesPerMeter = 360 / earthCircumference
)

func main() {
	lineStringFeature, err := readStreamToLineString(os.Stdin)
	if err != nil {
		log.Fatalln(err)
	}

	bwriter := bufio.NewWriter(os.Stdout)
	j, err := lineStringFeature.MarshalJSON()
	if err != nil {
		log.Fatalln(err)
	}
	if _, err := bwriter.Write(j); err != nil {
		log.Fatalln(err)
	}
	if err := bwriter.Flush(); err != nil {
		log.Fatalln(err)
	}
}

// func readStreamToMultiLineString(reader io.Reader) (*geojson.Feature, error) {
// 	breader := bufio.NewReader(reader)
//
// 	mls := orb.MultiLineString{}
// 	mlsf := geojson.NewFeature(mls)
//
// 	cursorName := ""
// 	cursorActivity := ""
// 	for {
// 		read, err := breader.ReadBytes('\n')
// 		if err != nil {
// 			if errors.Is(err, os.ErrClosed) || errors.Is(err, io.EOF) {
// 				break
// 			}
// 			log.Fatalln(err)
// 		}
//
// 		pointFeature, err := geojson.UnmarshalFeature(read)
// 		if err != nil {
// 			log.Fatalln(err)
// 		}
//
// 		cursorInit := cursorName == ""
//
// 		if cursorName != pointFeature.Properties["Name"].(string) {
// 			cursorName = pointFeature.Properties["Name"].(string)
// 			cursorActivity = pointFeature.Properties["Activity"].(string)
//
// 		}
//
// 		if cursorName == "" || cursorName != pointFeature.Properties["Name"].(string) {
// 			cursorName = pointFeature.Properties["Name"].(string)
// 			cursorActivity = pointFeature.Properties["Activity"].(string)
//
// 		} else if cursorActivity != pointFeature.Properties["Activity"].(string) {
// 			cursorActivity = pointFeature.Properties["Activity"].(string)
//
// 		}
//
// 		err = multiLineStringAddPoint(mlsf, pointFeature)
// 		if err != nil {
// 			log.Fatalln(err)
// 		}
// 	}
//
// 	return mlsf, nil
// }

func readStreamToLineString(reader io.Reader) (*geojson.Feature, error) {
	breader := bufio.NewReader(reader)

	ls := orb.LineString{}
	lsf := geojson.NewFeature(ls)

	for {
		read, err := breader.ReadBytes('\n')
		if err != nil {
			if errors.Is(err, os.ErrClosed) || errors.Is(err, io.EOF) {
				break
			}
			log.Fatalln(err)
		}

		pointFeature, err := geojson.UnmarshalFeature(read)
		if err != nil {
			log.Fatalln(err)
		}

		err = lineStringAddPoint(lsf, pointFeature)
		if err != nil {
			log.Fatalln(err)
		}
	}

	return lsf, nil
}

func mustGetTime(f *geojson.Feature, key string) time.Time {
	t := time.Time{}
	if _, ok := f.Properties[key]; !ok {
		return t
	}
	tt, err := time.Parse(time.RFC3339, f.Properties[key].(string))
	if err != nil {
		return t
	}
	return tt
}

// https://stackoverflow.com/questions/18390266/how-can-we-truncate-float64-type-to-a-particular-precision
func round(num float64) int {
	return int(num + math.Copysign(0.5, num))
}

func toFixed(num float64, precision int) float64 {
	output := math.Pow(10, float64(precision))
	return float64(round(num*output)) / output
}

type floatStats struct {
	Min  float64
	Max  float64
	Mean float64
}

func lineStringAddPoint(ls *geojson.Feature, point *geojson.Feature) error {
	_ls := ls.Geometry.(orb.LineString)
	_ls = append(_ls, point.Point())
	ls.Geometry = _ls

	// Overwriting properties. 3
	if v, ok := point.Properties["Name"]; ok {
		ls.Properties["Name"] = v
	}

	if v, ok := point.Properties["UUID"]; ok {
		ls.Properties["UUID"] = v
	}

	if _, ok := ls.Properties["Accuracies"]; ok {
		ls.Properties["Accuracies"] = append(ls.Properties["Accuracies"].([]float64), point.Properties["Accuracy"].(float64))
	} else {
		ls.Properties["Accuracies"] = []float64{point.Properties["Accuracy"].(float64)}
	}

	// Bounded properties.
	t := mustGetTime(point, "Time")

	if _, ok := ls.Properties["UnixTimes"]; ok {
		ls.Properties["UnixTimes"] = append(ls.Properties["UnixTimes"].([]int64), t.Unix())
	} else {
		ls.Properties["UnixTimes"] = []int64{t.Unix()}
	}

	lsStartTime := mustGetTime(ls, "StartTime")
	if lsStartTime.IsZero() || t.Before(lsStartTime) {
		ls.Properties["StartTime"] = t.Format(time.RFC3339)
		ls.Properties["StartTimeUnix"] = t.Unix()
	}

	lsEndTime := mustGetTime(ls, "EndTime")
	if lsEndTime.IsZero() || t.After(lsEndTime) {
		ls.Properties["EndTime"] = t.Format(time.RFC3339)
		ls.Properties["EndTimeUnix"] = t.Unix()
	}

	// Aggregate properties.
	if ls.Properties["Activities"] == nil {
		ls.Properties["Activities"] = map[string]int{}
	}

	if v, ok := point.Properties["Activity"]; ok && v != "" {
		if _, ok := ls.Properties["Activities"].(map[string]int)[v.(string)]; ok {
			ls.Properties["Activities"].(map[string]int)[v.(string)]++
		} else {
			ls.Properties["Activities"].(map[string]int)[v.(string)] = 1
		}
	}

	if _, ok := ls.Properties["Elevations"]; ok {
		ls.Properties["Elevations"] = append(ls.Properties["Elevations"].([]float64), toFixed(point.Properties["Elevation"].(float64), 0))
	} else {
		ls.Properties["Elevations"] = []float64{toFixed(point.Properties["Elevation"].(float64), 0)}
	}

	if _, ok := ls.Properties["Speeds"]; ok {
		ls.Properties["Speeds"] = append(ls.Properties["Speeds"].([]float64), toFixed(point.Properties["Speed"].(float64), 1))
	} else {
		ls.Properties["Speeds"] = []float64{toFixed(point.Properties["Speed"].(float64), 1)}
	}

	min, err := stats.Min(ls.Properties["Elevations"].([]float64))
	if err != nil {
		return err
	}
	max, err := stats.Max(ls.Properties["Elevations"].([]float64))
	if err != nil {
		return err
	}
	mean, err := stats.Mean(ls.Properties["Elevations"].([]float64))
	if err != nil {
		return err
	}

	if v, ok := ls.Properties["ElevationStats"]; ok {
		v := v.(map[string]float64)
		v["Min"] = toFixed(min, 1)
		v["Max"] = toFixed(max, 1)
		v["Mean"] = toFixed(mean, 1)
		ls.Properties["ElevationStats"] = v
	} else {
		ls.Properties["ElevationStats"] = map[string]float64{
			"Min":  toFixed(min, 1),
			"Max":  toFixed(max, 1),
			"Mean": toFixed(mean, 1),
		}
	}

	min, err = stats.Min(ls.Properties["Speeds"].([]float64))
	if err != nil {
		return err
	}
	max, err = stats.Max(ls.Properties["Speeds"].([]float64))
	if err != nil {
		return err
	}
	mean, err = stats.Mean(ls.Properties["Speeds"].([]float64))
	if err != nil {
		return err
	}

	if v, ok := ls.Properties["SpeedStats"]; ok {
		v := v.(map[string]float64)
		v["Min"] = toFixed(min, 1)
		v["Max"] = toFixed(max, 1)
		v["Mean"] = toFixed(mean, 1)
		ls.Properties["SpeedStats"] = v
	} else {
		ls.Properties["SpeedStats"] = map[string]float64{
			"Min":  toFixed(min, 1),
			"Max":  toFixed(max, 1),
			"Mean": toFixed(mean, 1),
		}
	}

	return nil
}

type Observation struct {
	Time     time.Time
	Point    mat.Vector
	Accuracy float64
}

func NewObservation(secondsOffset float64, x, y, accuracy float64) Observation {
	return Observation{
		Point:    mat.NewVecDense(2, []float64{x, y}),
		Time:     time.Time{}.Add(time.Duration(secondsOffset * float64(time.Second))),
		Accuracy: accuracy,
	}
}

func modifyKalman(ls *geojson.Feature) error {
	/*
		var t time.Time
		values := []float64{1.3, 10.2, 5.0, 3.4}

		model := models.NewSimpleModel(t, values[0], models.SimpleModelConfig{
			InitialVariance:     1.0,
			ProcessVariance:     1.0,
			ObservationVariance: 2.0,
		})
		models.NewConstantVelocityModel(t, &mat.VecDense{}, models.ConstantVelocityModelConfig{
			InitialVariance: 1.0,
			ProcessVariance: 1.0,
		})
		filter := kalman.NewKalmanFilter(model)

		for _, v := range values {
			t = t.Add(time.Second)
			filter.Update(t, model.NewMeasurement(v))
			fmt.Printf("filtered value: %f\n", model.Value(filter.State()))
		}
	*/
	if len(ls.Geometry.(orb.LineString)) < 2 {
		return errors.New("line string must have at least two points")
	}
	observations := []Observation{}
	for i, pt := range ls.Geometry.(orb.LineString) {
		y := pt[0]
		x := pt[1]
		secondsOffset := ls.Properties["UnixTimes"].([]int64)[i]
		accuracy := ls.Properties["Accuracies"].([]float64)[i] / earthCircumferenceDegreesPerMeter
		observations = append(observations, NewObservation(float64(secondsOffset), x, y, accuracy))
	}

	model := models.NewConstantVelocityModel(observations[0].Time, observations[0].Point, models.ConstantVelocityModelConfig{
		InitialVariance: 5 / earthCircumferenceDegreesPerMeter,
		ProcessVariance: 5 / earthCircumferenceDegreesPerMeter / 2,
	})

	filteredTrajectory, err := kalmanFilter(model, observations)
	if err != nil {
		return err
	}

	for i, fpt := range filteredTrajectory {
		ls.Geometry.(orb.LineString)[i][0] = fpt.AtVec(1) // y
		ls.Geometry.(orb.LineString)[i][1] = fpt.AtVec(0) // x
	}

	return nil
}

func kalmanFilter(model *models.ConstantVelocityModel, observations []Observation) ([]mat.Vector, error) {
	result := make([]mat.Vector, len(observations))
	filter := kalman.NewKalmanFilter(model)

	for i, obs := range observations {
		err := filter.Update(obs.Time, model.NewPositionMeasurement(obs.Point, obs.Accuracy))
		if err != nil {
			return nil, err
		}

		result[i] = model.Position(filter.State())
	}

	return result, nil
}
