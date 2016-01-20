#include <linux/module.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/imx.h>
#include <video/imx-ipu-v3.h>

struct ipu_media_controller {
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
};

static struct ipu_media_controller *ipu_media;

struct media_device *ipu_find_media_device(void)
{
	return &ipu_media->mdev;
}
EXPORT_SYMBOL_GPL(ipu_find_media_device);

struct ipu_media_link {
	struct v4l2_async_notifier	asn;
	struct v4l2_async_subdev	asd;
	struct v4l2_async_subdev	*asdp;
	struct v4l2_subdev		*sd;
	struct device_node		*node;
	int padno;
	struct device_node		*endpoint;
	u32				media_link_flags;
	bool				completed;
};

static int ipu_media_bound(struct v4l2_async_notifier *notifier,
			   struct v4l2_subdev *sd,
			   struct v4l2_async_subdev *asd)
{
	struct ipu_media_link		*link =
		container_of(notifier, struct ipu_media_link, asn);
	struct ipu_media_controller	*ctl =
		container_of(notifier->v4l2_dev, struct ipu_media_controller,
			     v4l2_dev);
	struct device_node *rp;
	struct property *prop;
	const __be32 *p;
	uint32_t link_num;
	uint32_t portno;
	int ret;
	bool have_links = false;

	rp = of_graph_get_remote_port(link->endpoint);
	if (of_property_read_u32(rp, "reg", &portno) < 0)
		portno = 0;

	ret = 0;
	of_property_for_each_u32(rp, "links", prop, p, link_num) {
		unsigned long		flags = link->media_link_flags;

		v4l2_info(link->sd, "creating link from pad %u\n", link_num);

		flags &= ~(MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

		if (link_num == portno)
			flags |= MEDIA_LNK_FL_ENABLED;

		ret = media_entity_create_link(&sd->entity, link_num,
					       &link->sd->entity, link->padno,
					       flags);
		if (ret)
			break;

		have_links = true;
	}

	of_node_put(rp);

	if (!ret && !have_links)
		ret = media_entity_create_link(&sd->entity, portno,
					       &link->sd->entity, link->padno,
					       link->media_link_flags);

	if (ret)
		return ret;

	ret = v4l2_device_register_subdev_node(&ctl->v4l2_dev, sd);
	if (ret < 0) {
		media_entity_remove_links(&sd->entity);
		return ret;
	}

	v4l2_info(link->sd, "bound and linked sensor '%s'\n", sd->name);

	return 0;
}

static void ipu_media_unbind(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *sd,
			    struct v4l2_async_subdev *asd)
{
	struct ipu_media_link *link = container_of(notifier,
						   struct ipu_media_link, asn);

	v4l2_info(link->sd, "unbinding sensor '%s'...\n", sd->name);

	if (sd->devnode) {
		video_unregister_device(sd->devnode);
		sd->devnode = NULL;
	}

	media_entity_remove_links(&sd->entity);
}

static int ipu_media_complete(struct v4l2_async_notifier *notifier)
{
	struct ipu_media_link		*link =
		container_of(notifier, struct ipu_media_link, asn);
	struct ipu_media_controller	*ctl =
		container_of(notifier->v4l2_dev, struct ipu_media_controller,
			     v4l2_dev);

	WARN_ON(link->completed);
	link->completed = true;

	return v4l2_device_register_subdev_node(&ctl->v4l2_dev, link->sd);
}

struct ipu_media_link *ipu_media_entity_create_link(struct v4l2_subdev *sd,
		int padno, struct device_node *endpoint,
		u32 media_link_flags)
{
	struct ipu_media_controller *im = ipu_media;
	struct ipu_media_link *link = NULL;
	int ret;
	struct device_node *rpp = NULL;

	rpp = of_graph_get_remote_port_parent(endpoint);
	if (!rpp) {
		ret = -EINVAL;
		goto out;
	}

	pr_info("%s: link on %s pad %d endpoint: %s remotenodeparent: %s\n",
		__func__, sd->name, padno, endpoint->full_name, rpp->full_name);
	if (!im) {
		ret = -ENODEV;
		goto out;
	}

	link = devm_kzalloc(sd->v4l2_dev->dev, sizeof(*link), GFP_KERNEL);
	if (!link) {
		ret = -ENOMEM;
		goto out;
	}

	link->sd = sd;
	link->node = rpp;
	link->padno = padno;
	link->endpoint = endpoint;
	link->media_link_flags = media_link_flags;

	link->asd.match_type = V4L2_ASYNC_MATCH_OF;
	link->asd.match.of.node = rpp;

	link->asdp = &link->asd;

	link->asn.bound = ipu_media_bound;
	link->asn.unbind = ipu_media_unbind;
	link->asn.complete = ipu_media_complete;
	link->asn.subdevs = &link->asdp;
	link->asn.num_subdevs = 1;
	link->asn.v4l2_dev = &im->v4l2_dev;

	ret = v4l2_async_notifier_register(&im->v4l2_dev, &link->asn);
	if (ret)
		goto out;

	ret = 0;

out:
	if (ret < 0) {
		of_node_put(endpoint);
		of_node_put(rpp);
		devm_kfree(sd->v4l2_dev->dev, link);
		link = ERR_PTR(ret);
	}

	return link;
}
EXPORT_SYMBOL_GPL(ipu_media_entity_create_link);

void ipu_media_entity_remove_link(struct ipu_media_link *link)
{
	v4l2_async_notifier_unregister(&link->asn);
	media_entity_remove_links(&link->sd->entity);

	of_node_put(link->node);
	of_node_put(link->endpoint);
}
EXPORT_SYMBOL_GPL(ipu_media_entity_remove_link);

struct v4l2_device *ipu_media_get_v4l2_dev(void)
{
	struct v4l2_device *v4l2_dev;

	if (!ipu_media)
		return NULL;

	v4l2_dev = &ipu_media->v4l2_dev;
	v4l2_device_get(v4l2_dev);
	return v4l2_dev;
}
EXPORT_SYMBOL_GPL(ipu_media_get_v4l2_dev);

void ipu_media_put_v4l2_dev(struct v4l2_device *v4l2_dev)
{
	struct module	*owner;

	if (!v4l2_dev)
		return;

	v4l2_device_put(v4l2_dev);
}
EXPORT_SYMBOL_GPL(ipu_media_put_v4l2_dev);

int ipu_media_device_register(struct device *dev, unsigned int num_links)
{
	struct media_device *mdev;
	int ret;

	if (ipu_media)
		return -EBUSY;

	if (!num_links)
		return -ENOENT;

	ipu_media = devm_kzalloc(dev, sizeof(*ipu_media), GFP_KERNEL);
	if (!ipu_media)
		return -ENOMEM;

	mdev = &ipu_media->mdev;

	mdev->dev = dev;

	strlcpy(mdev->model, "i.MX IPUv3", sizeof(mdev->model));

	ret = media_device_register(mdev);
	if (ret)
		return ret;

	ipu_media->v4l2_dev.mdev = mdev;

	ret = v4l2_device_register(mdev->dev, &ipu_media->v4l2_dev);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_media_device_register);

int ipu_media_device_unregister(struct device *dev)
{
	struct media_device *mdev;

	if (!ipu_media || ipu_media->mdev.dev != dev)
		return -EINVAL;

	mdev = &ipu_media->mdev;

	v4l2_device_unregister(&ipu_media->v4l2_dev);
	media_device_unregister(mdev);

	ipu_media = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(ipu_media_device_unregister);

MODULE_LICENSE("GPL");
