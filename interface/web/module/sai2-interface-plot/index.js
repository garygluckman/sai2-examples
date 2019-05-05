(function() {
    var template = document.currentScript.ownerDocument.querySelector('#template-plot');
    
    customElements.define('sai2-interface-plot', class extends HTMLElement {
        constructor() {
            super();
            this.template = template;
            this.plotting = false;
            this.x = [];
            this.y = [];
            this.i = 1;
            this.fetchDataCallback = this.fetchDataCallback.bind(this);
            this.chart = null;
        }

        fetchDataCallback() {
            this.x = [...this.x];
            this.x.push(this.i);

            this.y = [...this.y];
            this.y.push(this.i);

            this.i++;

            this.chart.setOption({
                xAxis: {data: this.x},
                series:[{
                    name:'y',
                    data: this.y
                }]
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

            let self = this;

            plot_button.innerHTML = this.plotting ? 'Stop' : 'Start';
    
            // populate keys list
            get_redis_all_keys().done(function(keys) {
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
                series: [
                    {
                        name: 'data',
                        type: 'line',
                        smooth: true,
                        symbol: 'none',
                        
                        data: []
                    }
                ]
            });


            // set up listeners
            plot_button.onclick = function(e) {
                self.plotting = !self.plotting;
                if (self.plotting) {
                    plot_button.innerHTML = 'Stop';

                    // determine rate. convert from sec -> ms
                    let query_rate = 1000 * parseFloat(query_rate_input.value);

                    // set up plot timer callback
                    self.plotIntervalID = setInterval(self.fetchDataCallback, query_rate);

                } else {
                    plot_button.innerHTML = 'Start';
                    
                    // clear plot timer callback
                    clearInterval(self.plotIntervalID);
                }
            };
            
            // append to document
            this.appendChild(template_node);
        }
    });
})();