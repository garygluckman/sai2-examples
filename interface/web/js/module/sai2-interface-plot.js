import { get_redis_val, get_redis_all_keys } from '../redis.js';

const template = document.createElement('template');
template.innerHTML = `
  <style>
    .sai2-interface-plot-top {
      display: flex;
      flex-direction: column;
    }
    .sai2-interface-plot-top .metadata {
      display: flex;
      flex-direction: row;
      justify-content: space-around;
      align-items: center;
    }

    .sai2-interface-plot-top .plot-div {
      flex: 1;
    }

  </style>
	<div class="sai2-interface-plot-top">
    <div class="metadata">
      <label>X:</label>
      <select class="x_key"></select>
      <label>Y:</label>
      <select class="y_key"></select>
      <label>Rate</label>
      <input class="query_rate" type="number" step="0.1">
      <button class="plot_button"></button>
      <label class="error-label" style="color:red;"><label>
    </div>
    <div class="plot-div" style="height: 400px;"></div>
	</div>
`;

    
customElements.define('sai2-interface-plot', class extends HTMLElement {
  constructor() {
    super();
    this.template = template;

    this.chart = null;
    this.chart_config = null;
    this.plotting = false;
    
    this.x_key = null;
    this.y_key = null;

    this.fetchDataCallback = this.fetchDataCallback.bind(this);
  }

  fetchDataCallback() {
    get_redis_val([this.x_key, this.y_key]).then(data => {
      // TODO: abort if x is not scalar valued
      // TODO: if y is a vector, how do you plot multiple series?

      // TODO: add a new series for each element if y is a vector
      this.chart_config.series[0].data.push([data[this.x_key], data[this.y_key]])
      this.chart.setOption(this.chart_config);
    });
  }

  connectedCallback() {
    // attributes
    this.plot_type = this.getAttribute('plotType') || 'scatter';

    // get DOM elements
    let template_node = this.template.content.cloneNode(true);
    let xkey_select = template_node.querySelector('.x_key');
    let ykey_select = template_node.querySelector('.y_key');
    let query_rate_input = template_node.querySelector('.query_rate');
    let plot_button = template_node.querySelector('.plot_button');
    let plot_div = template_node.querySelector('.plot-div');
    let error_label = template_node.querySelector('.error-label');

    plot_button.innerHTML = this.plotting ? 'Stop' : 'Start';
    plot_button.className = 'button-enable';

    // populate keys list
    get_redis_all_keys().then(keys => {
      for (let key of keys.values()) {
        let opt = document.createElement('option');
        opt.value = key;
        opt.innerHTML = key;
        xkey_select.append(opt);

        let opt2 = opt.cloneNode();
        opt2.innerHTML = key;
        ykey_select.append(opt2);
      }
    });

    // initialize empty plot
    // TODO: temporary
    this.chart = echarts.init(plot_div);
    this.chart_config = {
      xAxis: {},
      yAxis: {},
      toolbox: {
        feature: {
          saveAsImage: {
            title: 'Save Plot'
          }
        }
      },
      series: [{
        symbolSize: 10,
        type: 'scatter',
        data: []
      }]
    };
    this.chart.setOption(this.chart_config);
    
    // register event listeners
    xkey_select.onchange = e => {
      this.x_key = e.target.value;
    };

    ykey_select.onchange = e => {
      this.y_key = e.target.value;
    };

    plot_button.onclick = () => {
      if (!this.x_key || !this.y_key) {
        error_label.innerHTML = 'Please select the x and y keys to plot.';
        return;
      }

      error_label.innerHTML = '';

      this.plotting = !this.plotting;
      if (this.plotting) {
        xkey_select.disabled = true;
        ykey_select.disabled = true;
    
        this.chart.setOption(this.chart_config);

        plot_button.innerHTML = 'Stop';
        plot_button.className = 'button-disable';

        // determine rate. convert from sec -> ms
        let query_rate = 1000 * parseFloat(query_rate_input.value);

        // set up plot timer callback
        this.plotIntervalID = setInterval(this.fetchDataCallback, query_rate);

      } else {
        plot_button.innerHTML = 'Start';
        plot_button.className = 'button-enable';
        xkey_select.disabled = false;
        ykey_select.disabled = false;
            
        // clear plot timer callback
        clearInterval(this.plotIntervalID);
      }
    };
      
    // append to document
    this.appendChild(template_node);

    // HACK: echarts plot needs to manually told to resize
    // if ResizeObserver is implemented in more browsers
    // hook it up to plotly_div and call resize when plotly_div changes
    setTimeout(() => this.chart.resize(), 250);
  }

  adoptedCallback() {
    alert('test');
  }
});
