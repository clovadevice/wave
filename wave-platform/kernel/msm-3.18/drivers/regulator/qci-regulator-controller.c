#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

static struct regulator *vdd_3v3, *vdd_5v, *vdd_18v, *vdd_1v8_l4;

static int controller_probe(struct platform_device *pdev){
	int err = 0;

	pr_info("%s\n",__func__);

	struct device *dev = &pdev->dev;

	vdd_3v3 = devm_regulator_get(dev, "qci_vdd_3v3");
	if (IS_ERR(vdd_3v3)) {
		pr_err("qci_vdd_3v3 regulator get failed\n");
		err = PTR_ERR(vdd_3v3);
		vdd_3v3 = NULL;
		goto fail;
	}

	/*vdd_5v = devm_regulator_get(dev, "qci_vdd_5v");
	if (IS_ERR(vdd_5v)) {
		pr_err("qci_vdd_5v regulator get failed\n");
		err = PTR_ERR(vdd_5v);
		vdd_5v = NULL;
		goto fail;
	}*/

	/*vdd_18v = devm_regulator_get(dev, "qci_vdd_18v");
	if (IS_ERR(vdd_18v)) {
		pr_err("qci_vdd_18v regulator get failed\n");
		err = PTR_ERR(vdd_18v);
		vdd_18v = NULL;
		goto fail;
	}*/

	vdd_1v8_l4 = devm_regulator_get(dev, "8916_l4");
	if (IS_ERR(vdd_1v8_l4)) {
		pr_err("8916_l4 regulator get failed\n");
		err = PTR_ERR(vdd_1v8_l4);
		vdd_1v8_l4 = NULL;
		goto fail;
	}

	if (vdd_3v3) {
		err = regulator_enable(vdd_3v3);
		if (err < 0) {
			pr_err("qci_vdd_3v3 regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_5v) {
		err = regulator_enable(vdd_5v);
		if (err < 0) {
			pr_err("qci_vdd_5v regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_18v) {
		err = regulator_enable(vdd_18v);
		if (err < 0) {
			pr_err("qci_vdd_18v regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_1v8_l4) {
		err = regulator_enable(vdd_1v8_l4);
		if (err < 0) {
			pr_err("8916_l4 regulator enable failed\n");
			goto fail;
		}
	}
	return 0;

fail:
	return err;
}

static int controller_resume(struct device *dev)
{
	int err = 0;

	pr_info("%s\n",__func__);

	if (vdd_3v3) {
		err = regulator_enable(vdd_3v3);
		if (err < 0) {
			pr_err("qci_vdd_3v3 regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_5v) {
		err = regulator_enable(vdd_5v);
		if (err < 0) {
			pr_err("qci_vdd_5v regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_18v) {
		err = regulator_enable(vdd_18v);
		if (err < 0) {
			pr_err("qci_vdd_18v regulator enable failed\n");
			goto fail;
		}
	}
	return 0;

fail:
	return err;
}

static int controller_suspend(struct device *dev)
{
	int err = 0;

	pr_info("%s\n",__func__);

	if (vdd_3v3) {
		err = regulator_disable(vdd_3v3);
		if (err < 0) {
			pr_err("qci_vdd_3v3 regulator disable failed\n");
			goto fail;
		}
	}

	if (vdd_5v) {
		err = regulator_disable(vdd_5v);
		if (err < 0) {
			pr_err("qci_vdd_5v regulator disable failed\n");
			goto fail;
		}
	}

	if (vdd_18v) {
		err = regulator_disable(vdd_18v);
		if (err < 0) {
			pr_err("qci_vdd_18v regulator disable failed\n");
			goto fail;
		}
	}
	return 0;

fail:
	return err;
}

static struct of_device_id controller_match_table[] = {
	{.compatible = "qci,regulator_controller"},
	{},
};

static const struct dev_pm_ops controller_pm = {
	.resume  = controller_resume,
	.suspend = controller_suspend,
};

static struct platform_driver controller_driver = {
	.driver		= {
		.name			= "qci-regulator-controller",
		.owner			= THIS_MODULE,
		.pm				= &controller_pm,
		.of_match_table	= controller_match_table,
	},
	.probe		= controller_probe,
};

static int __init controller_init(void){
	return platform_driver_register(&controller_driver);
}

module_init(controller_init);
MODULE_DESCRIPTION("QCI Regulator Controller");
MODULE_LICENSE("Proprietary");
