ARG deploy_tag=salobj_v5.7.0

FROM lsstts/deploy-env:${deploy_tag}

LABEL maintainer Tiago Ribeiro <tribeiro@lsst.org>

ARG mtmount=0.1.dev68+gf5d71b5
ARG idl=1.1.3_4.8.0
ARG config=v0.2.0

RUN source /home/saluser/.setup.sh && \
    conda install -c lsstts ts-mtmount=${mtmount} ts-idl=${idl}

RUN git clone https://github.com/lsst-ts/ts_config_mttcs.git

WORKDIR /home/saluser/ts_config_mttcs
RUN git checkout ${config} -b ${config}

WORKDIR /home/saluser/

ENV TS_CONFIG_MTTCS_DIR=/home/saluser/ts_config_mttcs
ENV RUN_ARG=""

COPY setup.sh /home/saluser/.setup.sh
