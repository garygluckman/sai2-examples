import { get_redis_val, get_redis_all_keys } from '../redis.js';
import { registerWindowResizeCallback } from '../resize.js';

const template = document.createElement('template');
template.innerHTML = `
  <style>
    .sai2-interface-trajectory-select-top {
      display: flex;
      flex-direction: column;
      height: 95%;
    }

    .sai2-interface-trajectory-select-top .metadata {
      display: flex;
      align-items: baseline;
      flex-wrap: wrap;
      justify-content: space-around;
      flex: 1;
    }    

    .sai2-interface-trajectory-select-top .plots {
      flex: 9;
    }  
  </style>
	<div class="sai2-interface-trajectory-select-top">
    <div class="metadata">
      <button class="trajectory-add-pt-btn">Add Point</button>
      <button class="trajectory-get-btn">Get Trajectory</button>
      <button class="trajectory-run-btn">Run Trajectory</button>
      <select id="point-remover" class="chosen_select" multiple data-placeholder="Add a point..."></select>
    </div>
    <div class="grid-half">
      <div class="col traj-xy"></div>
      <div class="col traj-xz"></div>
    </div>
  </div>
`;
    
customElements.define('sai2-interface-trajectory-select', class extends HTMLElement {
  constructor() {
    super();
    this.template = template;

    this.xy_plot = null;
    this.xy_config = null;

    this.xz_plot = null;
    this.xz_config = null;

    this.points = { x: [], y: [], z: [], idx: [] };
    this.next_point_index = 0;
  }

  connectedCallback() {
    // get DOM elements
    let template_node = this.template.content.cloneNode(true);
    let xy_div = template_node.querySelector('.traj-xy');
    let xz_div = template_node.querySelector('.traj-xz');

    let addPointButton = template_node.querySelector('.trajectory-add-pt-btn');
    let getTrajectoryButton = template_node.querySelector('.trajectory-get-btn');
    let runTrajectoryButton = template_node.querySelector('.trajectory-run-btn');
    let pointSelect = template_node.querySelector('#point-remover');

    // parse 
    let xLim = JSON.parse(this.getAttribute('xLim'));
    let yLim = JSON.parse(this.getAttribute('yLim'));
    let zLim = JSON.parse(this.getAttribute('zLim'));


    // initialize template
    this.default_config = {
      grid: {},
      xAxis: { type:'value', name: 'x', min: xLim[0], max: xLim[1] }, 
      series: [],
      legend: { type: 'scroll' },
      toolbox: {
        top: 'bottom',
        left: 'right',
        feature: {
          saveAsImage: { title: 'Save Plot'},
          dataZoom: { title: { zoom: 'Box Zoom', back: 'Reset View' } }
        }
      },
      dataset: { 
        source: this.points,
      }
    };

    // initialize select
    $('#point-remover').chosen({ width: '100%' });

    // initialize empty plot & templates
    this.xy_plot = echarts.init(xy_div);
    this.xz_plot = echarts.init(xz_div);

    this.xy_config = {...this.default_config};
    this.xz_config = {...this.default_config};

    this.xy_config.tooltip = { 
      triggerOn: 'none',
      formatter: params => {
        return `Point ${this.points.idx[params.dataIndex]}<br>X: ${params.data[0].toFixed(2)}<br>Y: ${params.data[1].toFixed(2)}`;
      }
    };

    this.xz_config.tooltip = { 
      triggerOn: 'none',
      formatter: params => {
        return `Point ${this.points.idx[params.dataIndex]}<br>X: ${params.data[0].toFixed(2)}<br>Z: ${params.data[1].toFixed(2)}`;
      }
    };

    this.xy_config.series = [{
      id: 'xy',
      type: 'line',
      lineStyle: {type: 'dashed'},
      encode: { x: 'x', y: 'y' },
      symbolSize: 24
    }];

    this.xz_config.series = [{
      id: 'xz',
      type: 'line',
      lineStyle: {type: 'dashed'},
      encode: { x: 'x', y: 'z' },
      symbolSize: 24,
    }];

    this.xy_config.yAxis = { type:'value', name: 'y', min: yLim[0], max: yLim[1] };
    this.xz_config.yAxis = { type:'value', name: 'z', min: zLim[0], max: zLim[1] };

    this.xy_plot.setOption(this.xy_config);
    this.xz_plot.setOption(this.xz_config);

    // XXX: This routine is highly inefficient. Because we have two dependent plots,
    // if we move a point, we need to make sure we update the counterpart in the other graph
    // currently, this is naively done by forcing a reinitialization on ALL points on a single drag
    let init_drag = (xy_plot, xy_config, xz_plot, xz_config, points) => {
      xy_config.graphic = [];
      xz_config.graphic = [];
      for (let i = 0; i < points.x.length; i++) {
        let graphic_template = {
          type: 'circle',
          shape: { cx: 0, cy: 0, r: 10 },
          z: 100,
          invisible: true,
          draggable: true,
          onmouseover: () => {
            let showTipAction = {
              type: 'showTip',
              seriesIndex: 0, // the control points will always be at index 0
              dataIndex: i
            };
            xy_plot.dispatchAction(showTipAction);
            xz_plot.dispatchAction(showTipAction);
          },
          onmouseout: () => {
            let hideTipeAction = { type: 'hideTip' };
            xy_plot.dispatchAction(hideTipeAction);
            xz_plot.dispatchAction(hideTipeAction);
          }
        };

        let xy_graphic = {
          ...graphic_template,
          position: xy_plot.convertToPixel('grid', [points.x[i], points.y[i]]),

          ondrag: echarts.util.curry(function(i, xy_plot, xy_config, xz_plot, xz_config, points, params) {
            let pt = xy_plot.convertFromPixel('grid', this.position);
            points.x[i] = pt[0];
            points.y[i] = pt[1];
            init_drag(xy_plot, xy_config, xz_plot, xz_config, points);
            xy_plot.setOption(xy_config);
            xz_plot.setOption(xz_config);
          }, i, xy_plot, xy_config, xz_plot, xz_config, points)
        };

        let xz_graphic = {
          ...graphic_template,
          position: xz_plot.convertToPixel('grid', [points.x[i], points.z[i]]),
          ondrag: echarts.util.curry(function(i, xy_plot, xy_config, xz_plot, xz_config, points, params) {
            let pt = xz_plot.convertFromPixel('grid', this.position);
            points.x[i] = pt[0];
            points.z[i] = pt[1];
            init_drag(xy_plot, xy_config, xz_plot, xz_config, points);
            xy_plot.setOption(xy_config);
            xz_plot.setOption(xz_config);
          }, i, xy_plot, xy_config, xz_plot, xz_config, points)
        };

        xy_config.graphic.push(xy_graphic);
        xz_config.graphic.push(xz_graphic);
      } 
    };

    // set up event listeners
    registerWindowResizeCallback(() => {
      this.xy_plot.resize();
      this.xz_plot.resize();
      init_drag(this.xy_plot, this.xy_config, this.xz_plot, this.xz_config, this.points);
      this.xy_plot.setOption(this.xy_config);
      this.xz_plot.setOption(this.xz_config);
    });

    addPointButton.onclick = () => {
      this.points.x.push((xLim[0] + xLim[1]) / 2);
      this.points.y.push((yLim[0] + yLim[1]) / 2);
      this.points.z.push((zLim[0] + zLim[1]) / 2);
      this.points.idx.push(this.next_point_index);

      init_drag(this.xy_plot, this.xy_config, this.xz_plot, this.xz_config, this.points);
      this.xy_plot.setOption(this.xy_config);
      this.xz_plot.setOption(this.xz_config);

      let opt = document.createElement('option');
      opt.value = this.next_point_index;
      opt.innerHTML = 'Point ' + opt.value;
      opt.selected = true;
      pointSelect.append(opt);
      $('#point-remover').trigger("chosen:updated");

      this.next_point_index++;
    };

    // XXX: Known bug: remove the middle element
    pointSelect.onchange = e => {
      let options = []
      for (let option of e.target.selectedOptions) {
        options.push(parseInt(option.value))
      }

      for (let i = this.points.idx.length - 1; i >= 0; i--) {
        if (options.includes(this.points.idx[i]))
          continue;
        
        this.points.x.splice(i, 1);
        this.points.y.splice(i, 1);
        this.points.z.splice(i, 1);
        this.points.idx.splice(i, 1);
      }

      init_drag(this.xy_plot, this.xy_config, this.xz_plot, this.xz_config, this.points);
      this.xy_plot.setOption(this.xy_config);
      this.xz_plot.setOption(this.xz_config);
    };

    // append to document
    this.appendChild(template_node);

    // notify plots to resize, one after the other
    setTimeout(() => {
      this.xy_plot.resize();
      this.xz_plot.resize();

      init_drag(this.xy_plot, this.xy_config, this.xz_plot, this.xz_config, this.points);
      this.xy_plot.setOption(this.xy_config);
      this.xz_plot.setOption(this.xz_config);

    }, 250);
  }
});
