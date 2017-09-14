/*
 * q_taprio.c	Time Aware Priority Scheduler
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 * Authors:	Vinicius Costa Gomes <vinicius.gomes@intel.com>
 * 		Jesus Sanchez-Palencia <jesus.sanchez-palencia@intel.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#include "utils.h"
#include "tc_util.h"

#define CLOCKID_INVALID (-1)
static const struct static_clockid {
	const char *name;
	clockid_t clockid;
} clockids_sysv[] = {
	{ "REALTIME", CLOCK_REALTIME },
	{ "TAI", CLOCK_TAI },
	{ "BOOTTIME", CLOCK_BOOTTIME },
	{ "MONOTONIC", CLOCK_MONOTONIC },
	{ NULL }
};

static void explain(void)
{
	fprintf(stderr, "Usage: ... taprio clockid CLOCKID\n");
	fprintf(stderr, "                  [num_tc NUMBER] [map P0 P1 ...] ");
	fprintf(stderr, "                  [queues TC0 TC1 TC2 ...] ");
	fprintf(stderr, "                  [ [sched-file file] | [sched-row index cmd gate-mask interval] ] ");
	fprintf(stderr, "                  [base-time time] [extension-time time] [cycle-time time] ");
	fprintf(stderr, "                  [preemption TC0 TC1 TC2 ... ] ");
	fprintf(stderr, "\nCLOCKID must be a valid SYS-V id (i.e. CLOCK_TAI)");
	fprintf(stderr, "\n");
}

static void explain_clockid(const char *val)
{
	fprintf(stderr, "taprio: illegal value for \"clockid\": \"%s\".\n", val);
	fprintf(stderr, "It must be a valid SYS-V id (i.e. CLOCK_TAI)\n");
}

static int get_clockid(__s32 *val, const char *arg)
{
	const struct static_clockid *c;

	/* Drop the CLOCK_ prefix if that is being used. */
	if (strcasestr(arg, "CLOCK_") != NULL)
		arg += sizeof("CLOCK_") - 1;

	for (c = clockids_sysv; c->name; c++) {
		if (strcasecmp(c->name, arg) == 0) {
			*val = c->clockid;

			return 0;
		}
	}

	return -1;
}

static const char* get_clock_name(clockid_t clockid)
{
	const struct static_clockid *c;

	for (c = clockids_sysv; c->name; c++) {
		if (clockid == c->clockid)
			return c->name;
	}

	return "invalid";
}

static int str_to_entry_cmd(const char *str)
{
	if (strcmp(str, "S") == 0)
		return TC_TAPRIO_CMD_SET_GATES;

	if (strcmp(str, "H") == 0)
		return TC_TAPRIO_CMD_SET_AND_HOLD;

	if (strcmp(str, "R") == 0)
		return TC_TAPRIO_CMD_SET_AND_RELEASE;

	return -1;
}

static int add_sched_list(FILE *f, struct nlmsghdr *n)
{
	__u32 interval, gatemask, index = 0;
	char *cmd_str;
	__u8 cmd;
	int err;

	while (fscanf(f, "%ms %x %" PRIu32 "\n", &cmd_str, &gatemask, &interval) != EOF)  {
		struct rtattr *entry;

		err = str_to_entry_cmd(cmd_str);
		free(cmd_str);

		if (err < 0)
			return err;

		cmd = err;

		entry = addattr_nest(n, 1024, TCA_TAPRIO_SCHED_ENTRY);

		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_INDEX, &index, sizeof(index));
		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_CMD, &cmd, sizeof(cmd));
		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_GATE_MASK, &gatemask, sizeof(gatemask));
		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_INTERVAL, &interval, sizeof(interval));

		addattr_nest_end(n, entry);
	}

	return 0;
}

static void explain_sched_row(void)
{
	fprintf(stderr, "Usage: ... taprio ... sched-row <index> <cmd> <gate mask> <interval>\n");
}

static int taprio_parse_opt(struct qdisc_util *qu, int argc,
			    char **argv, struct nlmsghdr *n, const char *dev)
{
	__u32 entry_interval = 0, entry_gatemask = 0, entry_index = 0;
	__s64 base_time = 0, extension_time = 0, cycle_time = 0;
	__s32 clockid = CLOCKID_INVALID;
	struct tc_mqprio_qopt opt = { };
	unsigned int preemption = 0;
	FILE *sched_file = NULL;
	struct rtattr *tail;
	__u8 entry_cmd = 0;
	int err, idx;

	while (argc > 0) {
		idx = 0;
		if (strcmp(*argv, "num_tc") == 0) {
			NEXT_ARG();
			if (get_u8(&opt.num_tc, *argv, 10)) {
				fprintf(stderr, "Illegal \"num_tc\"\n");
				return -1;
			}
		} else if (strcmp(*argv, "map") == 0) {
			while (idx < TC_QOPT_MAX_QUEUE && NEXT_ARG_OK()) {
				NEXT_ARG();
				if (get_u8(&opt.prio_tc_map[idx], *argv, 10)) {
					PREV_ARG();
					break;
				}
				idx++;
			}
			for ( ; idx < TC_QOPT_MAX_QUEUE; idx++)
				opt.prio_tc_map[idx] = 0;
		} else if (strcmp(*argv, "queues") == 0) {
			char *tmp, *tok;

			while (idx < TC_QOPT_MAX_QUEUE && NEXT_ARG_OK()) {
				NEXT_ARG();

				tmp = strdup(*argv);
				if (!tmp)
					break;

				tok = strtok(tmp, "@");
				if (get_u16(&opt.count[idx], tok, 10)) {
					free(tmp);
					PREV_ARG();
					break;
				}
				tok = strtok(NULL, "@");
				if (get_u16(&opt.offset[idx], tok, 10)) {
					free(tmp);
					PREV_ARG();
					break;
				}
				free(tmp);
				idx++;
			}
		} else if (strcmp(*argv, "sched-file") == 0) {
			if (entry_index || entry_cmd || entry_gatemask || entry_interval) {
				fprintf(stderr,
					"taprio: specifying both \"sched-file\" and \"sched-row\" is not allowed\n");
				return -1;
			}
			NEXT_ARG();

			sched_file = fopen(*argv, "r");
			if (!sched_file) {
				break;
			}
		} else if (strcmp(*argv, "sched-row") == 0) {
			if (sched_file) {
				fprintf(stderr, "taprio: specifying both \"sched-file\" and \"sched-row\" is not allowed\n");
				return -1;
			}

			NEXT_ARG();

			if (get_u32(&entry_index, *argv, 10)) {
				explain_sched_row();
				return -1;
			}

			NEXT_ARG();
			err = str_to_entry_cmd(*argv);
			if (err < 0) {
				explain_sched_row();
				return  -1;
			}
			entry_cmd = err;

			NEXT_ARG();
			if (get_u32(&entry_gatemask, *argv, 0)) {
				explain_sched_row();
				return -1;
			}

			NEXT_ARG();
			if (get_u32(&entry_interval, *argv, 0)) {
				explain_sched_row();
				return -1;
			}
		} else if (strcmp(*argv, "base-time") == 0) {
			NEXT_ARG();
			if (get_s64(&base_time, *argv, 10)) {
				PREV_ARG();
				break;
			}
		} else if (strcmp(*argv, "cycle-time") == 0) {
			NEXT_ARG();
			if (get_s64(&cycle_time, *argv, 10)) {
				PREV_ARG();
				break;
			}
		} else if (strcmp(*argv, "extension-time") == 0) {
			NEXT_ARG();
			if (get_s64(&extension_time, *argv, 10)) {
				PREV_ARG();
				break;
			}
		} else if (strcmp(*argv, "preemption") == 0) {
			__u8 enable;
			int i;

			for (i = 0; i < opt.num_tc; i++) {
				NEXT_ARG();

				if (get_u8(&enable, *argv, 0)) {
					PREV_ARG();
					break;
				}

				preemption |= enable ? BIT(i) : 0;
			}
		} else if (strcmp(*argv, "clockid") == 0) {
			NEXT_ARG();
			if (clockid != CLOCKID_INVALID) {
				fprintf(stderr, "taprio: duplicate \"clockid\" specification\n");
				return -1;
			}
			if (get_clockid(&clockid, *argv)) {
				explain_clockid(*argv);
				return -1;
			}
		} else if (strcmp(*argv, "help") == 0) {
			explain();
			return -1;
		} else {
			fprintf(stderr, "Unknown argument\n");
			return -1;
		}
		argc--; argv++;
	}

	tail = NLMSG_TAIL(n);
	addattr_l(n, 1024, TCA_OPTIONS, NULL, 0);

	if (opt.num_tc > 0)
		addattr_l(n, 1024, TCA_TAPRIO_ATTR_PRIOMAP, &opt, sizeof(opt));

	if (preemption)
		addattr_l(n, 1024, TCA_TAPRIO_ATTR_PREEMPT_MASK, &preemption, sizeof(preemption));

	if (base_time)
		addattr_l(n, 1024, TCA_TAPRIO_ATTR_SCHED_BASE_TIME, &base_time, sizeof(base_time));

	if (cycle_time)
		addattr_l(n, 1024, TCA_TAPRIO_ATTR_SCHED_CYCLE_TIME, &cycle_time, sizeof(cycle_time));

	if (extension_time)
		addattr_l(n, 1024, TCA_TAPRIO_ATTR_SCHED_EXTENSION_TIME, &extension_time, sizeof(extension_time));

	addattr_l(n, 1024, TCA_TAPRIO_ATTR_SCHED_CLOCKID, &clockid, sizeof(clockid));

	if (sched_file) {
		struct rtattr *entry_list;
		entry_list = addattr_nest(n, 1024, TCA_TAPRIO_ATTR_SCHED_ENTRY_LIST | NLA_F_NESTED);

		err = add_sched_list(sched_file, n);
		if (err < 0) {
			fprintf(stderr, "Could not read sched list from file\n");
			return -1;
		}

		addattr_nest_end(n, entry_list);
	}

	if (entry_index || entry_cmd || entry_gatemask || entry_interval) {
		struct rtattr *entry, *entry_list;

		entry_list = addattr_nest(n, 1024, TCA_TAPRIO_ATTR_SCHED_SINGLE_ENTRY | NLA_F_NESTED);
		entry = addattr_nest(n, 1024, TCA_TAPRIO_SCHED_ENTRY);

		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_INDEX, &entry_index, sizeof(entry_index));
		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_CMD, &entry_cmd, sizeof(entry_cmd));
		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_GATE_MASK, &entry_gatemask, sizeof(entry_gatemask));
		addattr_l(n, 1024, TCA_TAPRIO_SCHED_ENTRY_INTERVAL, &entry_interval, sizeof(entry_interval));

		addattr_nest_end(n, entry);
		addattr_nest_end(n, entry_list);
	}

	tail->rta_len = (void *) NLMSG_TAIL(n) - (void *) tail;

	return 0;
}

static const char *command_to_str(__u8 cmd)
{
	switch (cmd) {
	case TC_TAPRIO_CMD_SET_GATES:
		return "S";
	case TC_TAPRIO_CMD_SET_AND_HOLD:
		return "H";
	case TC_TAPRIO_CMD_SET_AND_RELEASE:
		return "R";
	default:
		return "Invalid";
	}
}

static int print_sched_list(FILE *f, struct rtattr *list)
{
	struct rtattr *item;
	int rem;

	if (list == NULL)
		return 0;

	rem = RTA_PAYLOAD(list);

	for (item = RTA_DATA(list); RTA_OK(item, rem); item = RTA_NEXT(item, rem)) {
		struct rtattr *tb[TCA_TAPRIO_SCHED_ENTRY_MAX + 1];
		__u32 index = 0, gatemask = 0, interval = 0;
		__u8 command = 0;

		parse_rtattr_nested(tb, TCA_TAPRIO_SCHED_ENTRY_MAX, item);

		if (tb[TCA_TAPRIO_SCHED_ENTRY_INDEX])
			index = rta_getattr_u32(tb[TCA_TAPRIO_SCHED_ENTRY_INDEX]);

		if (tb[TCA_TAPRIO_SCHED_ENTRY_CMD])
			command = rta_getattr_u8(tb[TCA_TAPRIO_SCHED_ENTRY_CMD]);

		if (tb[TCA_TAPRIO_SCHED_ENTRY_GATE_MASK])
			gatemask = rta_getattr_u32(tb[TCA_TAPRIO_SCHED_ENTRY_GATE_MASK]);

		if (tb[TCA_TAPRIO_SCHED_ENTRY_INTERVAL])
			interval = rta_getattr_u32(tb[TCA_TAPRIO_SCHED_ENTRY_INTERVAL]);

		fprintf(f, "\n		index %u cmd %s gate-mask 0x%x interval %u", index,
			command_to_str(command), gatemask, interval);
	}

	return 0;
}

static int taprio_print_opt(struct qdisc_util *qu, FILE *f, struct rtattr *opt)
{
	struct rtattr *tb[TCA_TAPRIO_ATTR_MAX + 1];
	struct tc_mqprio_qopt *qopt = 0;
	__s64 cycle_time = 0, extension_time = 0, base_time = 0;
	__s32 clockid = CLOCKID_INVALID;
	__u32 preempt_mask = 0;
	int i;

	if (opt == NULL)
		return 0;

	parse_rtattr_nested(tb, TCA_TAPRIO_ATTR_MAX, opt);

	if (tb[TCA_TAPRIO_ATTR_PRIOMAP] == NULL)
		return -1;

	qopt = RTA_DATA(tb[TCA_TAPRIO_ATTR_PRIOMAP]);

	fprintf(f, "tc %u map ", qopt->num_tc);
	for (i = 0; i <= TC_PRIO_MAX; i++)
		fprintf(f, "%u ", qopt->prio_tc_map[i]);
	fprintf(f, "\n	queues:");
	for (i = 0; i < qopt->num_tc; i++)
		fprintf(f, "(%u:%u) ", qopt->offset[i],
			qopt->offset[i] + qopt->count[i] - 1);

	if (tb[TCA_TAPRIO_ATTR_SCHED_CYCLE_TIME])
		cycle_time = rta_getattr_s64(tb[TCA_TAPRIO_ATTR_SCHED_CYCLE_TIME]);

	if (tb[TCA_TAPRIO_ATTR_SCHED_EXTENSION_TIME])
		extension_time = rta_getattr_s64(tb[TCA_TAPRIO_ATTR_SCHED_EXTENSION_TIME]);

	if (tb[TCA_TAPRIO_ATTR_SCHED_BASE_TIME])
		base_time = rta_getattr_s64(tb[TCA_TAPRIO_ATTR_SCHED_BASE_TIME]);

	if (tb[TCA_TAPRIO_ATTR_PREEMPT_MASK])
		preempt_mask = rta_getattr_s64(tb[TCA_TAPRIO_ATTR_PREEMPT_MASK]);

	if (tb[TCA_TAPRIO_ATTR_SCHED_CLOCKID])
		clockid = rta_getattr_s32(tb[TCA_TAPRIO_ATTR_SCHED_CLOCKID]);

	fprintf(f, "\n	clockid %s ", get_clock_name(clockid));

	fprintf(f, "\n	base-time %lld cycle-time %lld extension-time %lld ",
		base_time, cycle_time, extension_time);

	fprintf(f, "\n	preempt-mask 0x%x ", preempt_mask);

	return print_sched_list(f, tb[TCA_TAPRIO_ATTR_SCHED_ENTRY_LIST]);
}

struct qdisc_util taprio_qdisc_util = {
	.id		= "taprio",
	.parse_qopt	= taprio_parse_opt,
	.print_qopt	= taprio_print_opt,
};
