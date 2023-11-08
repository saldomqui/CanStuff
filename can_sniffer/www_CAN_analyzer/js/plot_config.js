
/*
var value_plot_config = {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            {
                label: 'value',
                fill: false,
                borderColor: 'rgba(0,255,0,1)',
                backgroundColor: 'rgba(0,255,0,1)',
                data: []
            }]
    },
    options: {
        legend: {
            display: false
        },
        title: {
            display: false
        },
        scales: {
            xAxes: [
                {
                    gridLines: {
                        display: true,
                        color: "#aaaaaa",
                        lineWidth: 1,
                        zeroLineWidth: 3,
                        zeroLineColor: 'rgba(255,255,255,0.5)'
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Time'
                    }
                    //ticks: {
                    //   min: -5,
                    //   max: 5
                    //}
                }],
            yAxes: [
                {
                    type: 'linear',
                    display: true,
                    position: 'left',
                    scaleLabel: {
                        display: true,
                        labelString: 'Converted Value'
                    },
                    gridLines: {
                        display: true,
                        color: "#aaaaaa",
                        lineWidth: 1,
                        zeroLineWidth: 3,
                        zeroLineColor: 'rgba(255,255,255,0.5)'
                    }
                    //ticks: {
                    //    min: -5,
                    //    max: 15
                    //}
                }]
        }

    }
};

*/

var value_plot_config = {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            {
                label: '',
                fill: false,
                borderColor: 'rgba(0,255,0,1)',
                backgroundColor: 'rgba(0,255,0,1)',
                data: []
            }]
    },
    options: {
        legend: {
            position: 'top',
            labels: {
                fontColor: 'rgba(255,255,255,1)'
            }
        },
        title: {
            display: false,
            text: 'Converted value',
            fontColor: 'rgba(255,255,255,1)'
        },
        scales: {
            xAxes: [
                {
                    gridLines: {
                        display: true,
                        color: "#aaaaaa",
                        lineWidth: 1,
                        zeroLineWidth: 3,
                        zeroLineColor: 'rgba(255,255,255,0.5)'
                    },
                    scaleLabel: {
                        display: true,
                        labelString: 'Time'
                    }
                }],
            yAxes: [
                {
                    type: 'linear',
                    display: true,
                    position: 'left',
                    scaleLabel: {
                        display: true,
                        labelString: 'Converted value'
                    },
                    gridLines: {
                        display: true,
                        color: "#aaaaaa",
                        lineWidth: 1,
                        zeroLineWidth: 3,
                        zeroLineColor: 'rgba(255,255,255,0.5)'
                    }
                }],
        }

    }
};