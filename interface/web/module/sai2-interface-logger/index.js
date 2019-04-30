(function() {
var template = document.currentScript.ownerDocument.querySelector('#template-logger');

customElements.define('sai2-interface-logger', class extends HTMLElement {
	constructor() {
		super();
		this.template = template;
	}

	getLoggerStatus() {
		return $.ajax({
			method: "GET",
			url: "/logger/status",
		}).fail(function(data){
			alert('ajax error: ' + toString(data));
		});
	}

	connectedCallback() {
		let template_node = this.template.content.cloneNode(true);
		
		let button = template_node.querySelector('button');
		let logfile_input = template_node.querySelector('.logfile');
		let logperiod_input = template_node.querySelector('.logperiod');
		let keys_select = template_node.querySelector('select');

		let self = this;
		this.getLoggerStatus().done(function(status){
			self.logging = status['running'];
			button.innerHTML = self.logging ? 'stop logging' : 'start logging';
		});

		button.innerHTML = this.logging ? 'stop logging' : 'start logging';

		// populate keys list
		get_redis_all_keys().done(function(keys) {
			for (let key of keys.values()) {
				let opt = document.createElement('option');
				opt.value = key;
				opt.innerHTML = key;
				keys_select.append(opt);
			}
		});

		// set up listeners
		button.onclick = function(e) {
			self.logging = !self.logging;
			if (self.logging) {
				// default to log.txt
				let filename = logfile_input.value || 'log.txt';

				// get selected keys
				let selected_keys = [];
				for (let option of keys_select.options)
					if (option.selected)
						selected_keys.push(option.value);

				// get logger period. default to 0.1s
				let logger_period = logperiod_input.value || 0.1;

				self.start_logging(filename, selected_keys, logger_period)
					.done(function(data) {
						button.innerHTML = 'stop logging';
					});
			} else {
				console.log('stop logging');
				self.stop_logging()
					.done(function(data) {
						button.innerHTML = 'start logging';
					});
			}
		};
		
		// append to document
		this.appendChild(template_node);
	}

	start_logging(filename, key_list, period) {
		return $.ajax({
			method: "POST",
			url: "/logger/start",
			contentType: 'application/json; charset=utf-8',
			data: JSON.stringify({
				filename: filename,
				keys: key_list,
				logger_period: period
			})
		}).fail(function(data) {
			alert('log error: ' + toString(data));
		});
	}

	stop_logging() {
		return $.ajax({
			method: "POST",
			url: "/logger/stop"
		}).fail(function(data) {
			alert('log error: ' + toString(data));
		});
	}
});
})();