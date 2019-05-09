import { get_redis_val, get_redis_all_keys } from '../redis.js';

const template = document.createElement('template');
template.innerHTML = `
	<div>
    <div class='display_row'>
      <label>X:</label>
      <select class="x_key"></select>
      <label>Y:</label>
      <select class="y_key"></select>
      <label>Rate</label>
      <input class="query_rate" type="number" step="0.1">
      <button class="plot_button"></button>
    </div>
    <div class="plot-div"  style="width: 600px;height:400px;"></div>
	</div>
`;

    
customElements.define('sai2-interface-plot', class extends HTMLElement {
    constructor() {
      super();
      this.template = template;

      this.chart = null;
      this.plotting = false;
      
      this.x_key = null;
      this.y_key = null;
      this.x = [];
      this.y = [];

      this.fetchDataCallback = this.fetchDataCallback.bind(this);
    }

    fetchDataCallback() {
      get_redis_val([this.x_key, this.y_key]).then(data => {
        // TODO: abort if x is not scalar valued
        // TODO: if y is a vector, how do you plot multiple series?
        // TODO: what if I wanted to plot against time?
        this.x.push(data[this.x_key]);
        this.y.push(data[this.y_key]);

        this.chart.setOption({
          xAxis: {data: this.x},
          series:[{
            name:'y',
            data: this.y
          }]
        });
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

        plot_button.innerHTML = this.plotting ? 'Stop' : 'Start';

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

        this.chart.setOption({
            xAxis: {
              type: 'value',
              boundaryGap: false,
            },
            yAxis: {
              boundaryGap: [0, '50%'],
              type: 'value'
            },
            series: [{
              name: 'data',
              type: 'line',
              smooth: true,
              symbol: 'none',
              data: []
            }]
        });

        // set up listeners
        plot_button.onclick = () => {
          this.plotting = !this.plotting;
          if (this.plotting) {
            plot_button.innerHTML = 'Stop';

            // determine rate. convert from sec -> ms
            let query_rate = 1000 * parseFloat(query_rate_input.value);

            // set up plot timer callback
            this.plotIntervalID = setInterval(this.fetchDataCallback, query_rate);

          } else {
            plot_button.innerHTML = 'Start';
                
            // clear plot timer callback
            clearInterval(this.plotIntervalID);
          }
        };
        
        // append to document
        this.appendChild(template_node);
    }
});