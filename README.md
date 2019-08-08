openease/js
================

[![Build Status](https://travis-ci.org/navidJadid/openease_js.svg?branch=master)](https://travis-ci.org/navidJadid/openease_js)

Repository which contains the javascript code for the openease project.

Cannot be run by itself as of right now. Either build the [whole project](https://github.com/ease-crc/openease) with docker-compose (will pull the most recent docker-build from [`docker-hub`](https://hub.docker.com/r/openease/js)) or check out the [development project](https://github.com/navidJadid/openease_webserver_development).

Apart from that, tests will of course still run in CI-environment.

**TODO:**
- add CI-test suite & add following lines to script in package.json:  
    "lint": "eslint ./",  
    "pretest": "npm run lint",  
    "test": "jest"
- add test coverage
- add [coveralls](https://coveralls.io/) & test coverage badge
- add version badge
- add license badge
