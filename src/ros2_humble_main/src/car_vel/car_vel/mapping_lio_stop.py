"""Choose the LIO stop route for the body selected before mapping starts."""

from lzp_service.robot_body import BodyConfigStore


def topics_for_body(body):
    if body == 'xingji':
        return '/xingji/mapping/lio_stop', '/xingji/mapping/lio_resume'
    if body == 'gangben_zsl1':
        return '/gangben/mapping/lio_stop', '/gangben/mapping/lio_resume'
    raise ValueError('未知机器人本体，无法选择建图定位暂停话题')


def mapping_lio_topics():
    # Body switching is locked while mapping runs, so launch-time routing is
    # stable until this LIO process exits. Corrupt body config fails closed.
    return topics_for_body(BodyConfigStore().load()['selected'])
